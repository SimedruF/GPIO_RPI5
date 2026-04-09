/*
 * test_blade_detect.cpp - Blade Detection via GPIO Polling
 *
 * High-resolution blade-pass detection for turbine RPM measurement.
 * Uses direct GPIO polling via mmap (~10 ns per read) with
 * clock_gettime(CLOCK_MONOTONIC) for nanosecond-precision timestamps.
 *
 * Designed for:
 *   - Hall-effect or inductive proximity sensors (digital output)
 *   - Up to 400,000 RPM turbines
 *   - Blade-pass frequency up to ~67 kHz (10 blades @ 400k RPM)
 *
 * Sensor connection:
 *   GPIO22 = BLADE_SENSOR  (Header pin 15) - active LOW pulse per blade
 *   GPIO23 = INDEX_SENSOR  (Header pin 16) - one pulse per revolution (optional)
 *
 * The sensor output should be a clean digital signal (HIGH idle, LOW on blade).
 * If the sensor outputs analog, use a comparator (e.g., LM393) to digitize.
 *
 * Compile:
 *   g++ test_blade_detect.cpp -I.. -L.. -l:gpio_rpi5.a -o test_blade_detect -Wall -Wextra -O2
 *
 * Run:
 *   sudo ./test_blade_detect              # basic RPM measurement
 *   sudo ./test_blade_detect log          # log all events to CSV
 *   sudo ./test_blade_detect stress       # GPIO polling speed test (no sensor needed)
 */

#include "../gpio_rpi5.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <math.h>

/* ============================================================
 * Pin Definitions
 * ============================================================ */
#define BLADE_SENSOR  GPIO22   /* Blade-pass sensor (active LOW) */
#define INDEX_SENSOR  GPIO23   /* Index pulse: 1x per revolution (optional) */

/* ============================================================
 * Configuration
 * ============================================================ */
#define NUM_BLADES         10      /* Number of blades on the turbine */
#define MAX_RPM            400000  /* Maximum expected RPM */
#define DEBOUNCE_NS        2000    /* 2 µs debounce (prevents double-count) */
#define RPM_WINDOW_SIZE    64      /* Rolling average window for RPM */
#define MAX_LOG_EVENTS     100000  /* Maximum events to log in CSV mode */

/* Derived timing constants */
/* At 400k RPM, 10 blades: min period between pulses = ~15 µs = 15000 ns */
#define MIN_PULSE_PERIOD_NS  (60000000000ULL / ((uint64_t)MAX_RPM * NUM_BLADES))

/* ============================================================
 * Timestamp helpers
 * ============================================================ */
static inline uint64_t timestamp_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static inline double ns_to_ms(uint64_t ns)
{
    return (double)ns / 1000000.0;
}

static inline double ns_to_us(uint64_t ns)
{
    return (double)ns / 1000.0;
}

/* ============================================================
 * Blade event data
 * ============================================================ */
typedef struct {
    uint64_t timestamp_ns;   /* Absolute timestamp of rising/falling edge */
    uint32_t blade_count;    /* Cumulative blade count */
    double   rpm_instant;    /* Instantaneous RPM from this pulse */
    double   period_us;      /* Period since last pulse (µs) */
} blade_event_t;

/* Rolling RPM calculator */
typedef struct {
    double   periods[RPM_WINDOW_SIZE]; /* Recent pulse periods in ns */
    int      head;
    int      count;
    double   sum;
} rpm_filter_t;

static void rpm_filter_init(rpm_filter_t *f)
{
    memset(f, 0, sizeof(*f));
}

static void rpm_filter_add(rpm_filter_t *f, double period_ns)
{
    if (f->count >= RPM_WINDOW_SIZE)
        f->sum -= f->periods[f->head];
    else
        f->count++;

    f->periods[f->head] = period_ns;
    f->sum += period_ns;
    f->head = (f->head + 1) % RPM_WINDOW_SIZE;
}

static double rpm_filter_get_rpm(const rpm_filter_t *f)
{
    if (f->count == 0) return 0.0;
    double avg_period_ns = f->sum / f->count;
    if (avg_period_ns <= 0.0) return 0.0;
    /* RPM = 60 / (avg_period_seconds * NUM_BLADES) */
    return 60.0e9 / (avg_period_ns * NUM_BLADES);
}

static double rpm_filter_get_freq(const rpm_filter_t *f)
{
    if (f->count == 0) return 0.0;
    double avg_period_ns = f->sum / f->count;
    if (avg_period_ns <= 0.0) return 0.0;
    return 1.0e9 / avg_period_ns;
}

/* ============================================================
 * Globals
 * ============================================================ */
static volatile int g_running = 1;

static void signal_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

/* ============================================================
 * Test 1: GPIO Polling Speed Benchmark
 *
 * Measures raw polling throughput — how fast can we read a GPIO
 * pin using the mmap backend? This determines the maximum blade
 * frequency we can reliably detect.
 * ============================================================ */
static int test_polling_speed(void)
{
    printf("--- Test 1: GPIO Polling Speed Benchmark ---\n");
    printf("  Measuring pinread() throughput on GPIO%d...\n\n", BLADE_SENSOR);

    pin_t p = pinopen(BLADE_SENSOR, INPUT);
    if (p.mode == UNDEF)
    {
        fprintf(stderr, "  FAIL: Could not open GPIO%d\n", BLADE_SENSOR);
        return -1;
    }
    pinpull(BLADE_SENSOR, PULL_UP);

    /* Warm up */
    for (int i = 0; i < 1000; i++)
        pinread(BLADE_SENSOR);

    /* Benchmark: 1 million reads */
    const int NUM_READS = 1000000;
    uint64_t t_start = timestamp_ns();

    volatile int dummy = 0; /* prevent optimization */
    for (int i = 0; i < NUM_READS; i++)
        dummy = pinread(BLADE_SENSOR);

    uint64_t t_end = timestamp_ns();
    uint64_t elapsed_ns = t_end - t_start;

    double reads_per_sec = (double)NUM_READS / ((double)elapsed_ns / 1.0e9);
    double ns_per_read = (double)elapsed_ns / NUM_READS;

    printf("  Results:\n");
    printf("    %d reads in %.2f ms\n", NUM_READS, ns_to_ms(elapsed_ns));
    printf("    %.1f ns per pinread()  (%.2f MHz)\n", ns_per_read, reads_per_sec / 1.0e6);
    printf("\n");

    /* Calculate maximum detectable blade frequency */
    double max_blade_freq = reads_per_sec / 4.0; /* Need ~4 samples per pulse */
    double max_rpm_detectable = (max_blade_freq * 60.0) / NUM_BLADES;

    printf("  Maximum detectable frequencies:\n");
    printf("    Blade-pass:  %.0f Hz  (with 4 samples/pulse minimum)\n", max_blade_freq);
    printf("    Turbine RPM: %.0f RPM (with %d blades)\n", max_rpm_detectable, NUM_BLADES);
    printf("    Target:      %.0f Hz / %d RPM\n",
           (double)MAX_RPM * NUM_BLADES / 60.0, MAX_RPM);

    if (max_rpm_detectable >= MAX_RPM)
        printf("\n  PASS: Polling is fast enough for %d RPM target\n", MAX_RPM);
    else
        printf("\n  WARNING: Polling may be marginal. Consider interrupt-based approach.\n");

    /* Edge detection latency test */
    printf("\n  Edge detection timing test (measure consecutive read jitter)...\n");

    uint64_t latencies[1000];
    int edge_count = 0;
    uint64_t prev_time = timestamp_ns();
    int prev_val = pinread(BLADE_SENSOR);

    for (int i = 0; i < 500000 && edge_count < 1000; i++)
    {
        int val = pinread(BLADE_SENSOR);
        uint64_t now = timestamp_ns();

        if (val != prev_val)
        {
            latencies[edge_count++] = now - prev_time;
            prev_time = now;
        }
        prev_val = val;
    }

    if (edge_count > 2)
    {
        double sum = 0, min_l = 1e18, max_l = 0;
        for (int i = 1; i < edge_count; i++) /* skip first */
        {
            double l = (double)latencies[i];
            sum += l;
            if (l < min_l) min_l = l;
            if (l > max_l) max_l = l;
        }
        printf("    Detected %d edges\n", edge_count);
        printf("    Period: min=%.1f µs, avg=%.1f µs, max=%.1f µs\n",
               min_l / 1000.0, sum / (edge_count - 1) / 1000.0, max_l / 1000.0);
    }
    else
    {
        printf("    No edges detected (sensor not connected or signal is static)\n");
        printf("    This is OK for the benchmark — polling speed was measured above.\n");
    }

    pinclose(BLADE_SENSOR);
    printf("\n");
    return 0;
}

/* ============================================================
 * Test 2: RPM Measurement (Blade Detection)
 *
 * Polls the blade sensor GPIO at maximum speed, detects falling
 * edges (blade pass), computes instantaneous and averaged RPM.
 *
 * Run for a specified duration or until Ctrl+C.
 * ============================================================ */
static int test_rpm_measurement(int duration_sec, int log_csv)
{
    printf("--- Test 2: RPM Measurement ---\n");
    printf("  Blade sensor: GPIO%d (pin 15, active LOW)\n", BLADE_SENSOR);
    printf("  Number of blades: %d\n", NUM_BLADES);
    printf("  Duration: %d seconds (Ctrl+C to stop early)\n", duration_sec);
    printf("  Debounce: %d ns (%.1f µs)\n", DEBOUNCE_NS, DEBOUNCE_NS / 1000.0);
    if (log_csv) printf("  Logging to: blade_log.csv\n");
    printf("\n");

    pin_t p = pinopen(BLADE_SENSOR, INPUT);
    if (p.mode == UNDEF)
    {
        fprintf(stderr, "  FAIL: Could not open GPIO%d\n", BLADE_SENSOR);
        return -1;
    }
    pinpull(BLADE_SENSOR, PULL_UP); /* Pull-up, sensor pulls LOW */

    /* Optional: index sensor for per-revolution timing */
    int has_index = 0;
    pin_t pi = pinopen(INDEX_SENSOR, INPUT);
    if (pi.mode != UNDEF)
    {
        pinpull(INDEX_SENSOR, PULL_UP);
        has_index = 1;
        printf("  Index sensor: GPIO%d (pin 16) — detected\n", INDEX_SENSOR);
    }
    else
    {
        printf("  Index sensor: not available (GPIO%d not usable)\n", INDEX_SENSOR);
    }
    printf("\n");

    /* CSV log file */
    FILE *csv = NULL;
    if (log_csv)
    {
        csv = fopen("blade_log.csv", "w");
        if (csv)
        {
            fprintf(csv, "blade_count,timestamp_us,period_us,rpm_instant,rpm_avg\n");
        }
        else
        {
            fprintf(stderr, "  WARNING: Could not open blade_log.csv for writing\n");
        }
    }

    /* Detection loop */
    rpm_filter_t rpm_filt;
    rpm_filter_init(&rpm_filt);

    signal(SIGINT, signal_handler);

    uint64_t t_start = timestamp_ns();
    uint64_t t_end = t_start + (uint64_t)duration_sec * 1000000000ULL;
    uint64_t last_edge_ns = 0;
    uint64_t last_display_ns = t_start;
    uint32_t blade_count = 0;
    uint32_t rev_count = 0;
    uint32_t index_count = 0;
    int prev_blade = pinread(BLADE_SENSOR);
    int prev_index = has_index ? pinread(INDEX_SENSOR) : HIGH;

    printf("  %-10s  %-12s  %-10s  %-10s  %-12s  %-10s\n",
           "Time (s)", "Blades", "Rev", "Period µs", "RPM (inst)", "RPM (avg)");
    printf("  %-10s  %-12s  %-10s  %-10s  %-12s  %-10s\n",
           "--------", "------", "---", "---------", "----------", "---------");

    while (g_running)
    {
        uint64_t now = timestamp_ns();
        if (now >= t_end) break;

        int blade_val = pinread(BLADE_SENSOR);

        /* Detect falling edge (HIGH -> LOW = blade pass) */
        if (prev_blade == HIGH && blade_val == LOW)
        {
            /* Debounce check */
            if (last_edge_ns > 0 && (now - last_edge_ns) < DEBOUNCE_NS)
            {
                prev_blade = blade_val;
                continue;
            }

            blade_count++;

            if (last_edge_ns > 0)
            {
                uint64_t period_ns = now - last_edge_ns;
                double rpm_inst = 60.0e9 / ((double)period_ns * NUM_BLADES);

                rpm_filter_add(&rpm_filt, (double)period_ns);

                /* Log to CSV */
                if (csv && blade_count <= MAX_LOG_EVENTS)
                {
                    fprintf(csv, "%u,%.3f,%.3f,%.1f,%.1f\n",
                            blade_count,
                            (double)(now - t_start) / 1000.0,
                            ns_to_us(period_ns),
                            rpm_inst,
                            rpm_filter_get_rpm(&rpm_filt));
                }
            }

            last_edge_ns = now;
        }
        prev_blade = blade_val;

        /* Index sensor (1x per revolution) */
        if (has_index)
        {
            int idx_val = pinread(INDEX_SENSOR);
            if (prev_index == HIGH && idx_val == LOW)
            {
                index_count++;
                rev_count++;
            }
            prev_index = idx_val;
        }
        else
        {
            /* Estimate revolutions from blade count */
            rev_count = blade_count / NUM_BLADES;
        }

        /* Display update every 500 ms */
        if ((now - last_display_ns) >= 500000000ULL)
        {
            double elapsed_s = (double)(now - t_start) / 1.0e9;
            double rpm_avg = rpm_filter_get_rpm(&rpm_filt);
            double rpm_inst = 0;

            if (rpm_filt.count > 0)
            {
                int prev_idx = (rpm_filt.head - 1 + RPM_WINDOW_SIZE) % RPM_WINDOW_SIZE;
                double last_period = rpm_filt.periods[prev_idx];
                if (last_period > 0)
                    rpm_inst = 60.0e9 / (last_period * NUM_BLADES);
            }

            uint64_t last_period_ns = 0;
            if (rpm_filt.count > 0)
            {
                int prev_idx = (rpm_filt.head - 1 + RPM_WINDOW_SIZE) % RPM_WINDOW_SIZE;
                last_period_ns = (uint64_t)rpm_filt.periods[prev_idx];
            }

            printf("  %7.1f s   %-12u  %-10u  %8.1f    %10.0f    %10.0f\n",
                   elapsed_s, blade_count, rev_count,
                   ns_to_us(last_period_ns), rpm_inst, rpm_avg);

            last_display_ns = now;
        }
    }

    /* Final summary */
    uint64_t total_ns = timestamp_ns() - t_start;
    double total_s = (double)total_ns / 1.0e9;
    double avg_rpm = rpm_filter_get_rpm(&rpm_filt);

    printf("\n  === Summary ===\n");
    printf("    Duration:        %.2f seconds\n", total_s);
    printf("    Blade pulses:    %u\n", blade_count);
    printf("    Revolutions:     %u (estimated)\n", blade_count / NUM_BLADES);
    printf("    Avg blade freq:  %.1f Hz\n", blade_count / total_s);
    printf("    Avg RPM:         %.0f\n", avg_rpm);
    printf("    Blade rate:      %.0f pulses/sec\n", blade_count / total_s);

    if (csv)
    {
        fclose(csv);
        printf("    CSV log saved:   blade_log.csv (%u events)\n",
               blade_count < MAX_LOG_EVENTS ? blade_count : MAX_LOG_EVENTS);
    }

    if (has_index) pinclose(INDEX_SENSOR);
    pinclose(BLADE_SENSOR);

    if (blade_count == 0)
    {
        printf("\n  NOTE: No blade pulses detected.\n");
        printf("    - Check sensor wiring to GPIO%d (pin 15)\n", BLADE_SENSOR);
        printf("    - Verify sensor output is digital (LOW on blade pass)\n");
        printf("    - Use 'stress' mode to verify GPIO polling works\n");
    }

    printf("\n");
    return 0;
}

/* ============================================================
 * Test 3: Blade Timing Analysis
 *
 * Captures a burst of blade events and analyzes:
 * - Pulse-to-pulse jitter (indicates blade imbalance)
 * - Per-blade timing (detects individual blade anomalies)
 * - Frequency stability over time
 * ============================================================ */
static int test_blade_timing_analysis(void)
{
    printf("--- Test 3: Blade Timing Analysis ---\n");
    printf("  Capturing %d blade events for jitter analysis...\n", RPM_WINDOW_SIZE * NUM_BLADES);
    printf("  Sensor: GPIO%d (pin 15)\n\n", BLADE_SENSOR);

    pin_t p = pinopen(BLADE_SENSOR, INPUT);
    if (p.mode == UNDEF)
    {
        fprintf(stderr, "  FAIL: Could not open GPIO%d\n", BLADE_SENSOR);
        return -1;
    }
    pinpull(BLADE_SENSOR, PULL_UP);

    signal(SIGINT, signal_handler);

    int target_events = RPM_WINDOW_SIZE * NUM_BLADES;
    uint64_t *edge_times = (uint64_t *)malloc(target_events * sizeof(uint64_t));
    if (!edge_times)
    {
        fprintf(stderr, "  FAIL: Memory allocation\n");
        pinclose(BLADE_SENSOR);
        return -1;
    }

    int event_idx = 0;
    int prev_val = pinread(BLADE_SENSOR);
    uint64_t last_edge = 0;
    uint64_t timeout = timestamp_ns() + 10000000000ULL; /* 10 second timeout */

    printf("  Waiting for blade pulses (10 second timeout)...\n");

    while (g_running && event_idx < target_events)
    {
        if (timestamp_ns() > timeout)
        {
            printf("  Timeout reached with %d events captured.\n", event_idx);
            break;
        }

        int val = pinread(BLADE_SENSOR);

        if (prev_val == HIGH && val == LOW)
        {
            uint64_t now = timestamp_ns();
            if (last_edge > 0 && (now - last_edge) < DEBOUNCE_NS)
            {
                prev_val = val;
                continue;
            }
            edge_times[event_idx++] = now;
            last_edge = now;
        }
        prev_val = val;
    }

    if (event_idx < 3)
    {
        printf("  Not enough events captured (%d). Cannot analyze.\n", event_idx);
        printf("  Verify sensor connection and turbine is running.\n");
        free(edge_times);
        pinclose(BLADE_SENSOR);
        return -1;
    }

    printf("  Captured %d events. Analyzing...\n\n", event_idx);

    /* Calculate pulse-to-pulse periods */
    int num_periods = event_idx - 1;
    double *periods = (double *)malloc(num_periods * sizeof(double));
    if (!periods)
    {
        free(edge_times);
        pinclose(BLADE_SENSOR);
        return -1;
    }

    double sum = 0, min_p = 1e18, max_p = 0;
    for (int i = 0; i < num_periods; i++)
    {
        periods[i] = (double)(edge_times[i + 1] - edge_times[i]);
        sum += periods[i];
        if (periods[i] < min_p) min_p = periods[i];
        if (periods[i] > max_p) max_p = periods[i];
    }
    double mean_p = sum / num_periods;

    /* Standard deviation */
    double var_sum = 0;
    for (int i = 0; i < num_periods; i++)
    {
        double diff = periods[i] - mean_p;
        var_sum += diff * diff;
    }
    double stddev = sqrt(var_sum / num_periods);
    double jitter_pct = (stddev / mean_p) * 100.0;

    /* RPM stats */
    double rpm_from_mean = 60.0e9 / (mean_p * NUM_BLADES);

    printf("  Pulse Period Statistics:\n");
    printf("    Num periods:     %d\n", num_periods);
    printf("    Mean period:     %.3f µs\n", mean_p / 1000.0);
    printf("    Min period:      %.3f µs\n", min_p / 1000.0);
    printf("    Max period:      %.3f µs\n", max_p / 1000.0);
    printf("    Std deviation:   %.3f µs\n", stddev / 1000.0);
    printf("    Jitter:          %.3f %%\n", jitter_pct);
    printf("    Avg RPM:         %.0f\n", rpm_from_mean);
    printf("    Blade-pass freq: %.1f Hz\n", 1.0e9 / mean_p);
    printf("\n");

    /* Per-blade analysis (group by blade position within revolution) */
    if (num_periods >= NUM_BLADES)
    {
        printf("  Per-Blade Analysis (deviation from mean period):\n");
        printf("    %-8s  %-12s  %-10s\n", "Blade#", "Avg Period", "Deviation");
        printf("    %-8s  %-12s  %-10s\n", "------", "----------", "---------");

        for (int blade = 0; blade < NUM_BLADES; blade++)
        {
            double blade_sum = 0;
            int blade_count = 0;
            for (int i = blade; i < num_periods; i += NUM_BLADES)
            {
                blade_sum += periods[i];
                blade_count++;
            }
            if (blade_count > 0)
            {
                double blade_avg = blade_sum / blade_count;
                double deviation_pct = ((blade_avg - mean_p) / mean_p) * 100.0;
                printf("    Blade %-3d  %8.3f µs   %+.3f %%\n",
                       blade, blade_avg / 1000.0, deviation_pct);
            }
        }
        printf("\n");
        printf("  Note: Large deviations (>1%%) may indicate blade imbalance or damage.\n");
    }

    free(periods);
    free(edge_times);
    pinclose(BLADE_SENSOR);

    if (jitter_pct < 1.0)
        printf("\n  PASS: Jitter %.3f%% is within normal range\n", jitter_pct);
    else if (jitter_pct < 5.0)
        printf("\n  WARNING: Jitter %.3f%% — investigate blade condition\n", jitter_pct);
    else
        printf("\n  ALERT: Jitter %.3f%% is excessive — potential imbalance!\n", jitter_pct);

    printf("\n");
    return 0;
}

/* ============================================================ */

static void print_usage(const char *prog)
{
    printf("Usage: sudo %s [options]\n", prog);
    printf("Options:\n");
    printf("  (none)    RPM measurement for 10 seconds\n");
    printf("  log       RPM measurement + CSV logging\n");
    printf("  stress    GPIO polling speed benchmark (no sensor needed)\n");
    printf("  analyze   Capture & analyze blade timing jitter\n");
    printf("  all       Run all tests\n");
    printf("  help      Show this message\n");
}

int main(int argc, char *argv[])
{
    int result = 0;
    int do_stress  = 0;
    int do_measure = 0;
    int do_log     = 0;
    int do_analyze = 0;

    /* Parse arguments */
    if (argc < 2)
    {
        do_measure = 1; /* Default: 10 sec RPM measurement */
    }
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "stress") == 0)       do_stress = 1;
        else if (strcmp(argv[i], "log") == 0)   { do_measure = 1; do_log = 1; }
        else if (strcmp(argv[i], "analyze") == 0) do_analyze = 1;
        else if (strcmp(argv[i], "all") == 0)   { do_stress = 1; do_measure = 1; do_analyze = 1; }
        else if (strcmp(argv[i], "help") == 0 || strcmp(argv[i], "-h") == 0)
        {
            print_usage(argv[0]);
            return 0;
        }
        else
        {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    printf("===============================================\n");
    printf(" Blade Detection & RPM Measurement\n");
    printf(" GPIO_RPI5 High-Resolution Polling\n");
    printf("===============================================\n\n");

    if (gpio_init() < 0)
    {
        fprintf(stderr, "FATAL: gpio_init() failed. Are you root on a Raspberry Pi 5?\n");
        return 1;
    }

    /* Test 1: Polling speed benchmark */
    if (do_stress)
    {
        if (test_polling_speed() < 0)
            result = 1;
    }

    /* Test 2: RPM measurement */
    if (do_measure)
    {
        if (test_rpm_measurement(10, do_log) < 0)
            result = 1;
    }

    /* Test 3: Blade timing analysis */
    if (do_analyze)
    {
        if (test_blade_timing_analysis() < 0)
            result = 1;
    }

    gpio_cleanup();

    printf("=== %s ===\n", result ? "SOME TESTS FAILED" : "ALL TESTS PASSED");
    return result;
}
