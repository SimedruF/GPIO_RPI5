/*
 * test_ad7606.cpp - AD7606 8-Channel Simultaneous SAR ADC Test
 *
 * Test for AD7606 / AD7606B / AD7606C boards on Raspberry Pi 5
 * using the GPIO_RPI5 library (bit-banged SPI via mmap).
 *
 * AD7606 Key Specs:
 *   - 16-bit SAR ADC, 8 channels SIMULTANEOUS sampling
 *   - Up to 200 kSPS (all 8 channels converted at the same instant)
 *   - Bipolar input: ±5V (default) or ±10V (selectable via RANGE pin)
 *   - On-chip oversampling: 2x, 4x, 8x, 16x, 32x, 64x
 *   - SPI clock up to 23 MHz
 *   - Internal reference 2.5V
 *
 * AD7606 SPI Protocol:
 *   1. Pulse CONVST (rising edge) → starts simultaneous conversion
 *   2. Wait BUSY goes LOW (~3.5 µs) → conversion complete
 *   3. CS LOW → clock out 8 × 16-bit results (MSB first, 128 SCLKs)
 *   4. CS HIGH
 *
 * Pin Mapping (adjust to your board):
 *   GPIO11 = SCLK     (Header pin 23)
 *   GPIO10 = MOSI/DIN (Header pin 19)  — not used, AD7606 has no input data
 *   GPIO9  = MISO/DOUT(Header pin 21)  — AD7606 data output
 *   GPIO8  = CS       (Header pin 24)  — Chip Select (active LOW)
 *   GPIO17 = BUSY     (Header pin 11)  — Conversion busy (active HIGH)
 *   GPIO27 = CONVST   (Header pin 13)  — Convert Start (rising edge trigger)
 *   GPIO22 = RESET    (Header pin 15)  — Hardware reset (active HIGH)
 *   GPIO5  = OS0      (Header pin 29)  — Oversampling bit 0
 *   GPIO6  = OS1      (Header pin 31)  — Oversampling bit 1
 *   GPIO13 = OS2      (Header pin 33)  — Oversampling bit 2
 *   GPIO26 = RANGE    (Header pin 37)  — 0=±5V, 1=±10V
 *
 * NOTE: Pin assignments may differ between board manufacturers.
 *       Adjust the #defines below to match YOUR specific wiring.
 *
 * Compile:
 *   g++ test_ad7606.cpp -I.. -L.. -l:gpio_rpi5.a -o test_ad7606 -Wall -Wextra -O2 -lm
 *
 * Run:
 *   sudo ./test_ad7606              # read all 8 channels once
 *   sudo ./test_ad7606 rate         # benchmark conversion rate
 *   sudo ./test_ad7606 cont         # continuous capture (Ctrl+C to stop)
 *   sudo ./test_ad7606 oversample   # test all oversampling modes
 *   sudo ./test_ad7606 capture      # capture 1 sec to CSV
 *   sudo ./test_ad7606 all          # run all tests
 */

#include "../gpio_rpi5.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

/* ============================================================
 * Pin Definitions — ADJUST TO YOUR WIRING
 *
 * If using Waveshare AD/DA Board, check their schematic for
 * exact GPIO assignments. Common configurations below.
 * ============================================================ */

/* SPI Data Pins */
#define AD_SCLK    GPIO11   /* SPI Clock */
#define AD_MISO    GPIO9    /* Data Out from AD7606 (DOUT/DB7) */
#define AD_CS      GPIO8    /* Chip Select (directly active LOW) */

/* Control Pins */
#define AD_CONVST  GPIO27   /* Convert Start A+B (active rising edge) */
#define AD_BUSY    GPIO17   /* Busy indicator (HIGH during conversion) */
#define AD_RESET   GPIO22   /* Reset (active HIGH pulse) */

/* Oversampling Configuration Pins */
#define AD_OS0     GPIO5    /* Oversampling bit 0 */
#define AD_OS1     GPIO6    /* Oversampling bit 1 */
#define AD_OS2     GPIO13   /* Oversampling bit 2 */

/* Range Selection */
#define AD_RANGE   GPIO26   /* LOW = ±5V,  HIGH = ±10V */

/* ============================================================
 * AD7606 Constants
 * ============================================================ */
#define AD7606_NUM_CH        8       /* Always 8 simultaneous channels */
#define AD7606_RESOLUTION    16      /* 16-bit ADC */
#define AD7606_VREF          2.5     /* Internal reference voltage */

/* Full-scale voltage for each range */
#define RANGE_5V             5.0     /* ±5V bipolar */
#define RANGE_10V            10.0    /* ±10V bipolar */

/* Timing (from datasheet) */
#define CONV_TIME_US         4       /* Max conversion time: 3.5 µs typ */
#define RESET_PULSE_US       50      /* Reset pulse width: min 50 ns, use 50 µs */
#define RESET_RECOVER_US     1000    /* Wait after reset */
#define BUSY_TIMEOUT_US      100000  /* 100 ms timeout for BUSY */

/* Oversampling ratios */
#define OS_NONE   0   /* No oversampling (200 kSPS) */
#define OS_2X     1   /* 2x  oversampling */
#define OS_4X     2   /* 4x  oversampling */
#define OS_8X     3   /* 8x  oversampling */
#define OS_16X    4   /* 16x oversampling */
#define OS_32X    5   /* 32x oversampling */
#define OS_64X    6   /* 64x oversampling */

/* Max capture buffer */
#define MAX_CAPTURE_SAMPLES  200000  /* 1 second at 200 kSPS */

/* ============================================================
 * Timestamp helpers
 * ============================================================ */
static inline uint64_t time_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

/* ============================================================
 * Globals
 * ============================================================ */
static volatile int g_running = 1;
static double g_full_scale = RANGE_5V;  /* Active range */

static void signal_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

/* ============================================================
 * AD7606 Low-Level Driver
 * ============================================================ */

/*
 * Fast SPI read — clocks in 16 bits from AD7606 DOUT.
 *
 * AD7606 SPI is output-only (no MOSI needed for reading).
 * Data valid on SCLK falling edge (CPOL=0, CPHA=1 equivalent).
 * MSB first, 16-bit signed (two's complement).
 *
 * With mmap GPIO at ~10 ns/operation, we achieve ~6 MHz+ effective SCLK.
 */
static int16_t ad7606_spi_read16(void)
{
    uint16_t val = 0;

    for (int i = 15; i >= 0; i--)
    {
        pinwrite(AD_SCLK, HIGH);
        /* Data is valid on falling edge */
        pinwrite(AD_SCLK, LOW);
        if (pinread(AD_MISO))
            val |= (1U << i);
    }

    return (int16_t)val;
}

/* Initialize all AD7606 pins */
static int ad7606_gpio_init(void)
{
    pin_t p;

    /* SPI pins */
    p = pinopen(AD_SCLK, OUTPUT);
    if (p.mode == UNDEF) { fprintf(stderr, "FAIL: GPIO%d (SCLK)\n", AD_SCLK); return -1; }
    pinwrite(AD_SCLK, LOW);

    p = pinopen(AD_MISO, INPUT);
    if (p.mode == UNDEF) { fprintf(stderr, "FAIL: GPIO%d (MISO)\n", AD_MISO); return -1; }
    pinpull(AD_MISO, PULL_NONE);

    p = pinopen(AD_CS, OUTPUT);
    if (p.mode == UNDEF) { fprintf(stderr, "FAIL: GPIO%d (CS)\n", AD_CS); return -1; }
    pinwrite(AD_CS, HIGH);

    /* Control pins */
    p = pinopen(AD_CONVST, OUTPUT);
    if (p.mode == UNDEF) { fprintf(stderr, "FAIL: GPIO%d (CONVST)\n", AD_CONVST); return -1; }
    pinwrite(AD_CONVST, LOW);

    p = pinopen(AD_BUSY, INPUT);
    if (p.mode == UNDEF) { fprintf(stderr, "FAIL: GPIO%d (BUSY)\n", AD_BUSY); return -1; }
    pinpull(AD_BUSY, PULL_DOWN);

    p = pinopen(AD_RESET, OUTPUT);
    if (p.mode == UNDEF) { fprintf(stderr, "FAIL: GPIO%d (RESET)\n", AD_RESET); return -1; }
    pinwrite(AD_RESET, LOW);

    /* Oversampling pins */
    p = pinopen(AD_OS0, OUTPUT);
    if (p.mode == UNDEF) { fprintf(stderr, "FAIL: GPIO%d (OS0)\n", AD_OS0); return -1; }
    p = pinopen(AD_OS1, OUTPUT);
    if (p.mode == UNDEF) { fprintf(stderr, "FAIL: GPIO%d (OS1)\n", AD_OS1); return -1; }
    p = pinopen(AD_OS2, OUTPUT);
    if (p.mode == UNDEF) { fprintf(stderr, "FAIL: GPIO%d (OS2)\n", AD_OS2); return -1; }

    /* Range pin */
    p = pinopen(AD_RANGE, OUTPUT);
    if (p.mode == UNDEF) { fprintf(stderr, "FAIL: GPIO%d (RANGE)\n", AD_RANGE); return -1; }

    return 0;
}

/* Close all AD7606 pins */
static void ad7606_gpio_cleanup(void)
{
    pinclose(AD_SCLK);
    pinclose(AD_MISO);
    pinclose(AD_CS);
    pinclose(AD_CONVST);
    pinclose(AD_BUSY);
    pinclose(AD_RESET);
    pinclose(AD_OS0);
    pinclose(AD_OS1);
    pinclose(AD_OS2);
    pinclose(AD_RANGE);
}

/* Hardware reset */
static void ad7606_reset(void)
{
    pinwrite(AD_RESET, HIGH);
    usleep(RESET_PULSE_US);
    pinwrite(AD_RESET, LOW);
    usleep(RESET_RECOVER_US);
}

/* Set oversampling mode (0=none, 1=2x, 2=4x, 3=8x, 4=16x, 5=32x, 6=64x) */
static void ad7606_set_oversampling(int os_mode)
{
    pinwrite(AD_OS0, (os_mode >> 0) & 1);
    pinwrite(AD_OS1, (os_mode >> 1) & 1);
    pinwrite(AD_OS2, (os_mode >> 2) & 1);
    usleep(10); /* Allow pins to settle */
}

/* Set input range: 0 = ±5V, 1 = ±10V */
static void ad7606_set_range(int range_10v)
{
    pinwrite(AD_RANGE, range_10v ? HIGH : LOW);
    g_full_scale = range_10v ? RANGE_10V : RANGE_5V;
    usleep(10);
}

/* Wait for BUSY to go LOW (conversion complete) — busy-poll for speed */
static int ad7606_wait_busy(void)
{
    uint64_t deadline = time_ns() + (uint64_t)BUSY_TIMEOUT_US * 1000ULL;
    while (pinread(AD_BUSY) == HIGH)
    {
        if (time_ns() > deadline)
            return -1; /* Timeout */
    }
    return 0;
}

/*
 * Start a conversion and read all 8 channels.
 *
 * Sequence:
 *   1. Pulse CONVST HIGH → starts simultaneous conversion on all 8 channels
 *   2. Wait for BUSY LOW → conversion complete (~3.5 µs)
 *   3. CS LOW, clock out 8 × 16-bit (128 SCLK cycles)
 *   4. CS HIGH
 *
 * Returns 0 on success, fills data[0..7] with signed 16-bit codes.
 */
static int ad7606_read_all(int16_t data[AD7606_NUM_CH])
{
    /* Trigger conversion: rising edge on CONVST */
    pinwrite(AD_CONVST, HIGH);
    /* Hold CONVST HIGH briefly — min 25 ns, mmap write is ~10 ns */
    pinwrite(AD_CONVST, LOW);

    /* Wait for conversion to complete */
    if (ad7606_wait_busy() < 0)
        return -1;

    /* Read results via SPI */
    pinwrite(AD_CS, LOW);

    for (int ch = 0; ch < AD7606_NUM_CH; ch++)
        data[ch] = ad7606_spi_read16();

    pinwrite(AD_CS, HIGH);

    return 0;
}

/* Convert raw 16-bit signed code to voltage */
static double ad7606_to_voltage(int16_t raw)
{
    /*
     * AD7606 output: 16-bit two's complement
     *   0x7FFF = +Full Scale = +RANGE
     *   0x0000 = 0V
     *   0x8000 = -Full Scale = -RANGE
     *
     * Voltage = raw × RANGE / 32768
     */
    return (double)raw * g_full_scale / 32768.0;
}

/* Full AD7606 initialization */
static int ad7606_init(int range_10v, int os_mode)
{
    printf("[AD7606] Initializing...\n");

    if (ad7606_gpio_init() < 0)
    {
        fprintf(stderr, "[AD7606] FAIL: GPIO initialization\n");
        return -1;
    }

    /* Hardware reset */
    ad7606_reset();

    /* Configure */
    ad7606_set_range(range_10v);
    ad7606_set_oversampling(os_mode);

    /* Do a dummy conversion to clear the pipeline */
    int16_t dummy[AD7606_NUM_CH];
    if (ad7606_read_all(dummy) < 0)
    {
        fprintf(stderr, "[AD7606] WARNING: First dummy conversion timed out\n");
        fprintf(stderr, "  Check wiring:\n");
        fprintf(stderr, "    SCLK   -> GPIO%d (pin 23)\n", AD_SCLK);
        fprintf(stderr, "    DOUT   -> GPIO%d (pin 21)\n", AD_MISO);
        fprintf(stderr, "    CS     -> GPIO%d (pin 24)\n", AD_CS);
        fprintf(stderr, "    CONVST -> GPIO%d (pin 13)\n", AD_CONVST);
        fprintf(stderr, "    BUSY   -> GPIO%d (pin 11)\n", AD_BUSY);
        fprintf(stderr, "    RESET  -> GPIO%d (pin 15)\n", AD_RESET);
        return -1;
    }

    printf("[AD7606] Ready.\n");
    printf("  Range:         ±%.0fV\n", g_full_scale);
    printf("  Oversampling:  %dx\n", (os_mode == 0) ? 1 : (1 << os_mode));
    printf("  Channels:      %d (simultaneous)\n", AD7606_NUM_CH);
    printf("  Resolution:    %d-bit\n\n", AD7606_RESOLUTION);

    return 0;
}

/* ============================================================
 * Test 1: Read All Channels Once
 * ============================================================ */
static int test_read_once(void)
{
    printf("--- Test 1: Read All 8 Channels ---\n\n");

    int16_t data[AD7606_NUM_CH];

    if (ad7606_read_all(data) < 0)
    {
        fprintf(stderr, "  FAIL: Conversion timed out (BUSY stuck HIGH)\n");
        return -1;
    }

    printf("  %-8s  %-10s  %-12s  %-10s\n", "Channel", "Raw Code", "Voltage (V)", "% FS");
    printf("  %-8s  %-10s  %-12s  %-10s\n", "-------", "--------", "-----------", "-----");

    for (int ch = 0; ch < AD7606_NUM_CH; ch++)
    {
        double v = ad7606_to_voltage(data[ch]);
        double pct = (double)data[ch] / 327.68; /* % of full scale */
        printf("  V%-7d  %6d      %+8.4f      %+6.2f%%\n", ch + 1, data[ch], v, pct);
    }

    /* Sanity check: if all channels read 0x0000 or 0xFFFF, something is wrong */
    int all_zero = 1, all_ff = 1;
    for (int ch = 0; ch < AD7606_NUM_CH; ch++)
    {
        if (data[ch] != 0) all_zero = 0;
        if (data[ch] != -1 && data[ch] != (int16_t)0xFFFF) all_ff = 0;
    }

    printf("\n");
    if (all_zero)
        printf("  WARNING: All channels read 0 — inputs may be floating or MISO stuck LOW\n");
    else if (all_ff)
        printf("  WARNING: All channels read 0xFFFF — MISO may be stuck HIGH\n");
    else
        printf("  PASS: Data received from all channels\n");

    printf("\n");
    return 0;
}

/* ============================================================
 * Test 2: Conversion Rate Benchmark
 *
 * Measures actual achievable sample rate with bit-banged SPI.
 * ============================================================ */
static int test_rate_benchmark(void)
{
    printf("--- Test 2: Conversion Rate Benchmark ---\n");
    printf("  Target: 200,000 SPS (all 8ch simultaneous)\n\n");

    int16_t data[AD7606_NUM_CH];
    const int NUM_CONV = 10000;
    int successes = 0;
    int timeouts = 0;

    /* Warm up */
    for (int i = 0; i < 10; i++)
        ad7606_read_all(data);

    uint64_t t_start = time_ns();

    for (int i = 0; i < NUM_CONV; i++)
    {
        if (ad7606_read_all(data) == 0)
            successes++;
        else
            timeouts++;
    }

    uint64_t t_end = time_ns();
    double elapsed_s = (double)(t_end - t_start) / 1.0e9;
    double actual_sps = successes / elapsed_s;
    double time_per_conv_us = elapsed_s * 1.0e6 / successes;
    double total_sps = actual_sps * AD7606_NUM_CH; /* Effective channel-SPS */

    printf("  Results:\n");
    printf("    Conversions:     %d requested, %d ok, %d timeout\n",
           NUM_CONV, successes, timeouts);
    printf("    Elapsed:         %.3f seconds\n", elapsed_s);
    printf("    Samples/sec:     %.0f SPS (all 8ch per sample)\n", actual_sps);
    printf("    Effective rate:  %.0f channel-SPS (8ch × %.0f)\n", total_sps, actual_sps);
    printf("    Time/conversion: %.1f µs\n", time_per_conv_us);
    printf("    Breakdown est.:  ~3.5 µs conversion + ~%.1f µs SPI readout\n",
           time_per_conv_us - 3.5);
    printf("\n");

    /* Rate analysis */
    double pct_of_target = (actual_sps / 200000.0) * 100.0;
    printf("    Efficiency: %.1f%% of 200 kSPS target\n", pct_of_target);

    if (pct_of_target >= 90.0)
        printf("    PASS: Achieving >90%% of target rate\n");
    else if (pct_of_target >= 50.0)
        printf("    GOOD: >50%% — usable for most vibration analysis\n");
    else
        printf("    NOTE: Bit-bang SPI limits throughput. Use /dev/spidev for max rate.\n");

    /* Nyquist frequency analysis */
    double nyquist = actual_sps / 2.0;
    double blade_freq_400k = (400000.0 / 60.0) * 10; /* 10 blades at 400k RPM */
    printf("\n    Nyquist:     %.0f Hz\n", nyquist);
    printf("    Blade freq:  %.0f Hz (10 blades @ 400k RPM)\n", blade_freq_400k);
    if (nyquist >= blade_freq_400k)
        printf("    PASS: Can capture blade-pass frequency\n");
    else
        printf("    WARNING: Nyquist < blade frequency. Need hardware SPI for full speed.\n");

    printf("\n");
    return 0;
}

/* ============================================================
 * Test 3: Oversampling Modes
 *
 * Tests each oversampling setting and measures the resulting
 * conversion time and noise characteristics.
 * ============================================================ */
static int test_oversampling(void)
{
    printf("--- Test 3: Oversampling Modes ---\n\n");

    static const struct { int mode; const char *label; int ratio; } os_modes[] = {
        { OS_NONE, "None (1x)", 1 },
        { OS_2X,   "2x",  2 },
        { OS_4X,   "4x",  4 },
        { OS_8X,   "8x",  8 },
        { OS_16X,  "16x", 16 },
        { OS_32X,  "32x", 32 },
        { OS_64X,  "64x", 64 },
    };

    printf("  %-10s  %-10s  %-12s  %-12s  %-14s\n",
           "Mode", "Ratio", "Conv µs", "Rate (SPS)", "V1 StdDev (V)");
    printf("  %-10s  %-10s  %-12s  %-12s  %-14s\n",
           "--------", "-----", "-------", "----------", "-------------");

    for (int m = 0; m < 7; m++)
    {
        ad7606_set_oversampling(os_modes[m].mode);
        usleep(1000); /* Let oversampling config settle */

        /* Measure conversion time */
        int16_t data[AD7606_NUM_CH];
        const int N = 500;
        uint64_t t_start = time_ns();
        int ok = 0;

        for (int i = 0; i < N; i++)
        {
            if (ad7606_read_all(data) == 0) ok++;
        }

        uint64_t t_end = time_ns();
        double elapsed_s = (double)(t_end - t_start) / 1.0e9;
        double conv_us = elapsed_s * 1.0e6 / ok;
        double rate = ok / elapsed_s;

        /* Noise measurement: take 100 samples of CH1 and compute stddev */
        double sum = 0, sum_sq = 0;
        int noise_n = 0;

        for (int i = 0; i < 100; i++)
        {
            if (ad7606_read_all(data) == 0)
            {
                double v = ad7606_to_voltage(data[0]);
                sum += v;
                sum_sq += v * v;
                noise_n++;
            }
        }

        double mean = (noise_n > 0) ? sum / noise_n : 0;
        double var = (noise_n > 1) ? (sum_sq / noise_n - mean * mean) : 0;
        double stddev = (var > 0) ? sqrt(var) : 0;

        printf("  %-10s  %4dx       %8.1f     %10.0f     %.6f\n",
               os_modes[m].label, os_modes[m].ratio, conv_us, rate, stddev);
    }

    /* Restore to no oversampling */
    ad7606_set_oversampling(OS_NONE);

    printf("\n  Note: Higher oversampling reduces noise but also reduces max sample rate.\n");
    printf("  Choose based on your Nyquist requirement vs. noise floor needs.\n\n");
    return 0;
}

/* ============================================================
 * Test 4: Multi-Channel Statistics
 *
 * Captures multiple conversions and computes per-channel stats.
 * ============================================================ */
static int test_channel_stats(void)
{
    printf("--- Test 4: Multi-Channel Statistics (1000 samples) ---\n\n");

    const int N = 1000;
    int16_t data[AD7606_NUM_CH];

    /* Per-channel accumulators */
    double ch_sum[AD7606_NUM_CH] = {};
    double ch_sum_sq[AD7606_NUM_CH] = {};
    double ch_min[AD7606_NUM_CH];
    double ch_max[AD7606_NUM_CH];
    int    samples = 0;

    for (int ch = 0; ch < AD7606_NUM_CH; ch++)
    {
        ch_min[ch] = 1e18;
        ch_max[ch] = -1e18;
    }

    for (int i = 0; i < N; i++)
    {
        if (ad7606_read_all(data) < 0) continue;
        samples++;

        for (int ch = 0; ch < AD7606_NUM_CH; ch++)
        {
            double v = ad7606_to_voltage(data[ch]);
            ch_sum[ch] += v;
            ch_sum_sq[ch] += v * v;
            if (v < ch_min[ch]) ch_min[ch] = v;
            if (v > ch_max[ch]) ch_max[ch] = v;
        }
    }

    if (samples == 0)
    {
        fprintf(stderr, "  FAIL: No successful conversions\n\n");
        return -1;
    }

    printf("  %d samples captured.\n\n", samples);
    printf("  %-6s  %-10s  %-10s  %-10s  %-10s  %-10s\n",
           "Ch", "Mean (V)", "Min (V)", "Max (V)", "Vpp (V)", "StdDev (V)");
    printf("  %-6s  %-10s  %-10s  %-10s  %-10s  %-10s\n",
           "----", "--------", "-------", "-------", "-------", "---------");

    for (int ch = 0; ch < AD7606_NUM_CH; ch++)
    {
        double mean = ch_sum[ch] / samples;
        double var = ch_sum_sq[ch] / samples - mean * mean;
        double stddev = (var > 0) ? sqrt(var) : 0;
        double vpp = ch_max[ch] - ch_min[ch];

        printf("  V%-5d  %+8.4f    %+8.4f    %+8.4f    %8.4f    %.6f\n",
               ch + 1, mean, ch_min[ch], ch_max[ch], vpp, stddev);
    }

    printf("\n");
    return 0;
}

/* ============================================================
 * Test 5: Continuous Capture to CSV
 *
 * High-speed capture of all 8 channels for a specified duration.
 * Saves timestamped data to CSV for offline analysis (FFT, etc.)
 * ============================================================ */
typedef struct {
    int16_t  ch[AD7606_NUM_CH];
    uint64_t timestamp_ns;
} ad7606_sample_t;

static int test_capture_csv(int duration_sec)
{
    int max_samples = MAX_CAPTURE_SAMPLES;
    /* Estimate: at ~50 kSPS bit-banged, 1 sec ≈ 50k samples */
    if (duration_sec > 1) max_samples = MAX_CAPTURE_SAMPLES; /* cap at 200k */

    printf("--- Test 5: Continuous Capture to CSV ---\n");
    printf("  Duration: %d second(s), buffer: %d samples max\n\n", duration_sec, max_samples);

    ad7606_sample_t *buf = (ad7606_sample_t *)malloc(max_samples * sizeof(ad7606_sample_t));
    if (!buf)
    {
        fprintf(stderr, "  FAIL: Could not allocate %.1f MB buffer\n",
                max_samples * sizeof(ad7606_sample_t) / 1.0e6);
        return -1;
    }

    signal(SIGINT, signal_handler);

    uint64_t t_start = time_ns();
    uint64_t t_end = t_start + (uint64_t)duration_sec * 1000000000ULL;
    int count = 0;
    int timeouts = 0;

    printf("  Capturing... ");
    fflush(stdout);

    while (g_running && count < max_samples && time_ns() < t_end)
    {
        int16_t data[AD7606_NUM_CH];
        if (ad7606_read_all(data) < 0)
        {
            timeouts++;
            if (timeouts > 100) break;
            continue;
        }

        buf[count].timestamp_ns = time_ns();
        memcpy(buf[count].ch, data, sizeof(data));
        count++;

        if (count % (max_samples / 10) == 0)
        {
            printf("%d%% ", (count * 100) / max_samples);
            fflush(stdout);
        }
    }

    uint64_t capture_end = time_ns();
    double elapsed = (double)(capture_end - t_start) / 1.0e9;
    double rate = count / elapsed;

    printf("\n\n");
    printf("  Capture complete:\n");
    printf("    Samples:    %d\n", count);
    printf("    Timeouts:   %d\n", timeouts);
    printf("    Duration:   %.3f s\n", elapsed);
    printf("    Rate:       %.0f SPS (×8ch = %.0f channel-SPS)\n", rate, rate * 8);
    printf("\n");

    /* Write CSV */
    if (count > 0)
    {
        const char *filename = "ad7606_capture.csv";
        FILE *f = fopen(filename, "w");
        if (f)
        {
            fprintf(f, "sample,time_us,V1,V2,V3,V4,V5,V6,V7,V8\n");
            for (int i = 0; i < count; i++)
            {
                double t_us = (double)(buf[i].timestamp_ns - buf[0].timestamp_ns) / 1000.0;
                fprintf(f, "%d,%.3f", i, t_us);
                for (int ch = 0; ch < AD7606_NUM_CH; ch++)
                    fprintf(f, ",%.6f", ad7606_to_voltage(buf[i].ch[ch]));
                fprintf(f, "\n");
            }
            fclose(f);
            printf("  CSV saved: %s (%d rows)\n", filename, count);
        }
        else
        {
            fprintf(stderr, "  WARNING: Could not write %s\n", filename);
        }

        /* Quick stats on captured data */
        printf("\n  Quick channel summary (captured data):\n");
        for (int ch = 0; ch < AD7606_NUM_CH; ch++)
        {
            double sum = 0, min_v = 1e18, max_v = -1e18;
            for (int i = 0; i < count; i++)
            {
                double v = ad7606_to_voltage(buf[i].ch[ch]);
                sum += v;
                if (v < min_v) min_v = v;
                if (v > max_v) max_v = v;
            }
            double mean = sum / count;
            printf("    V%d: mean=%+.4f  Vpp=%.4f\n", ch + 1, mean, max_v - min_v);
        }
    }

    free(buf);
    printf("\n");
    return 0;
}

/* ============================================================
 * Test 6: Continuous Display (oscilloscope mode)
 * ============================================================ */
static int test_continuous(int duration_sec)
{
    printf("--- Test 6: Continuous Read (Ctrl+C to stop) ---\n\n");

    signal(SIGINT, signal_handler);

    uint64_t t_start = time_ns();
    uint64_t t_end = t_start + (uint64_t)duration_sec * 1000000000ULL;
    uint64_t last_display = t_start;
    int total_samples = 0;

    /* Running stats for display */
    double ch_sum[AD7606_NUM_CH] = {};
    int stats_count = 0;

    while (g_running && time_ns() < t_end)
    {
        int16_t data[AD7606_NUM_CH];
        if (ad7606_read_all(data) < 0) continue;
        total_samples++;

        for (int ch = 0; ch < AD7606_NUM_CH; ch++)
            ch_sum[ch] += ad7606_to_voltage(data[ch]);
        stats_count++;

        /* Display update every 200 ms */
        uint64_t now = time_ns();
        if ((now - last_display) >= 200000000ULL)
        {
            double elapsed = (double)(now - t_start) / 1.0e9;
            double rate = total_samples / elapsed;

            printf("\r  T=%5.1fs  %6.0f SPS | ", elapsed, rate);
            for (int ch = 0; ch < AD7606_NUM_CH; ch++)
            {
                double avg = (stats_count > 0) ? ch_sum[ch] / stats_count : 0;
                printf("V%d:%+6.3f ", ch + 1, avg);
            }
            fflush(stdout);

            /* Reset running stats */
            memset(ch_sum, 0, sizeof(ch_sum));
            stats_count = 0;
            last_display = now;
        }
    }

    double total_s = (double)(time_ns() - t_start) / 1.0e9;
    printf("\n\n  Captured %d samples in %.1f s (%.0f SPS)\n\n",
           total_samples, total_s, total_samples / total_s);
    return 0;
}

/* ============================================================
 * Test 7: Simultaneous Sampling Verification
 *
 * Verifies that all 8 channels are truly sampled at the same
 * instant (key advantage of AD7606 over multiplexed ADCs).
 * ============================================================ */
static int test_simultaneous(void)
{
    printf("--- Test 7: Simultaneous Sampling Verification ---\n");
    printf("  All 8 channels capture at the SAME instant.\n");
    printf("  Apply the same signal to multiple channels to verify.\n\n");

    const int N = 100;
    int16_t data[AD7606_NUM_CH];
    double ch_vals[AD7606_NUM_CH][100];
    int count = 0;

    for (int i = 0; i < N; i++)
    {
        if (ad7606_read_all(data) < 0) continue;
        for (int ch = 0; ch < AD7606_NUM_CH; ch++)
            ch_vals[ch][count] = ad7606_to_voltage(data[ch]);
        count++;
    }

    if (count < 10)
    {
        fprintf(stderr, "  FAIL: Not enough samples (%d)\n\n", count);
        return -1;
    }

    /* Cross-channel correlation: compare adjacent channels */
    printf("  Cross-channel analysis (%d samples):\n", count);
    printf("  %-12s  %-12s  %-12s\n", "Ch Pair", "Correlation", "Max Diff (V)");
    printf("  %-12s  %-12s  %-12s\n", "-------", "-----------", "-----------");

    for (int a = 0; a < AD7606_NUM_CH - 1; a++)
    {
        int b = a + 1;
        double sum_a = 0, sum_b = 0, sum_ab = 0, sum_a2 = 0, sum_b2 = 0;
        double max_diff = 0;

        for (int i = 0; i < count; i++)
        {
            sum_a += ch_vals[a][i];
            sum_b += ch_vals[b][i];
            sum_ab += ch_vals[a][i] * ch_vals[b][i];
            sum_a2 += ch_vals[a][i] * ch_vals[a][i];
            sum_b2 += ch_vals[b][i] * ch_vals[b][i];
            double d = fabs(ch_vals[a][i] - ch_vals[b][i]);
            if (d > max_diff) max_diff = d;
        }

        double mean_a = sum_a / count;
        double mean_b = sum_b / count;
        double var_a = sum_a2 / count - mean_a * mean_a;
        double var_b = sum_b2 / count - mean_b * mean_b;
        double cov = sum_ab / count - mean_a * mean_b;

        double corr = 0;
        if (var_a > 0 && var_b > 0)
            corr = cov / (sqrt(var_a) * sqrt(var_b));

        printf("  V%d vs V%d     %+.6f      %.6f\n", a + 1, b + 1, corr, max_diff);
    }

    printf("\n  If the same signal is applied to all channels:\n");
    printf("    Correlation should be ~1.000 (simultaneous = same reading)\n");
    printf("    Max diff should be < 1 LSB (%.6f V at ±%.0fV range)\n",
           g_full_scale / 32768.0, g_full_scale);

    printf("\n");
    return 0;
}

/* ============================================================ */

static void print_usage(const char *prog)
{
    printf("Usage: sudo %s [options]\n", prog);
    printf("Options:\n");
    printf("  (none)       Read all 8 channels once\n");
    printf("  rate         Benchmark conversion rate\n");
    printf("  oversample   Test all oversampling modes\n");
    printf("  stats        Multi-channel statistics (1000 samples)\n");
    printf("  capture      Capture 1 second to CSV\n");
    printf("  cont         Continuous display (10 sec, Ctrl+C)\n");
    printf("  verify       Simultaneous sampling verification\n");
    printf("  range10      Use ±10V range (default ±5V)\n");
    printf("  all          Run all tests\n");
    printf("  help         Show this message\n");
}

int main(int argc, char *argv[])
{
    int result = 0;
    int do_read = 0, do_rate = 0, do_os = 0;
    int do_stats = 0, do_capture = 0, do_cont = 0, do_verify = 0;
    int range_10v = 0;

    if (argc < 2)
        do_read = 1;

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "rate") == 0)          do_rate = 1;
        else if (strcmp(argv[i], "oversample") == 0) do_os = 1;
        else if (strcmp(argv[i], "stats") == 0)    do_stats = 1;
        else if (strcmp(argv[i], "capture") == 0)  do_capture = 1;
        else if (strcmp(argv[i], "cont") == 0)     do_cont = 1;
        else if (strcmp(argv[i], "verify") == 0)   do_verify = 1;
        else if (strcmp(argv[i], "range10") == 0)  range_10v = 1;
        else if (strcmp(argv[i], "all") == 0)
        {
            do_read = 1; do_rate = 1; do_os = 1;
            do_stats = 1; do_capture = 1; do_verify = 1;
        }
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

    printf("═══════════════════════════════════════════════════════\n");
    printf("  AD7606 8-Channel 16-Bit Simultaneous SAR ADC Test\n");
    printf("  GPIO_RPI5 Bit-Banged SPI\n");
    printf("═══════════════════════════════════════════════════════\n\n");

    if (gpio_init() < 0)
    {
        fprintf(stderr, "FATAL: gpio_init() failed. Are you root on a Raspberry Pi 5?\n");
        return 1;
    }

    if (ad7606_init(range_10v, OS_NONE) < 0)
    {
        fprintf(stderr, "FATAL: AD7606 initialization failed.\n");
        ad7606_gpio_cleanup();
        gpio_cleanup();
        return 1;
    }

    if (do_read)
    {
        if (test_read_once() < 0) result = 1;
    }
    if (do_rate)
    {
        if (test_rate_benchmark() < 0) result = 1;
    }
    if (do_os)
    {
        if (test_oversampling() < 0) result = 1;
        /* Restore no oversampling for subsequent tests */
        ad7606_set_oversampling(OS_NONE);
    }
    if (do_stats)
    {
        if (test_channel_stats() < 0) result = 1;
    }
    if (do_capture)
    {
        if (test_capture_csv(1) < 0) result = 1;
    }
    if (do_verify)
    {
        if (test_simultaneous() < 0) result = 1;
    }
    if (do_cont)
    {
        test_continuous(10);
    }

    ad7606_gpio_cleanup();
    gpio_cleanup();

    printf("=== %s ===\n", result ? "SOME TESTS FAILED" : "ALL TESTS PASSED");
    return result;
}
