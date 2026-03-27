/*
 * test_vibration.cpp - Vibration Monitoring via ADS1263 at Max Speed
 *
 * Continuous vibration data acquisition using the ADS1263 (ADC1, 32-bit)
 * at the maximum data rate of 38,400 SPS. Designed for low-frequency
 * vibration analysis on turbine/rotary equipment.
 *
 * The ADS1263 at 38,400 SPS provides:
 *   - Usable bandwidth: 0 - 19,200 Hz (Nyquist)
 *   - ENOB: ~18 bits (effective resolution)
 *   - Suitable for: structural vibration, low-speed bearing faults
 *   - NOT suitable for: blade-pass frequencies at 400k RPM (~67 kHz)
 *
 * Sensor connection (Waveshare High-Precision ADC HAT):
 *   AIN0 = Vibration sensor (accelerometer / piezo output)
 *   AIN1 = Second vibration axis (optional)
 *   SPI0 on GPIO8-11, DRDY=GPIO17, RST=GPIO18
 *
 * Compile:
 *   g++ test_vibration.cpp -I.. -L.. -l:gpio_rpi5.a -o test_vibration -Wall -Wextra -O2 -lm
 *
 * Run:
 *   sudo ./test_vibration                # capture 1 second, print stats
 *   sudo ./test_vibration fft            # capture + basic spectral analysis
 *   sudo ./test_vibration log            # capture + save raw data to CSV
 *   sudo ./test_vibration dual           # capture AIN0 + AIN1 interleaved
 *   sudo ./test_vibration all            # run all tests
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
 * Pin Definitions (Waveshare High-Precision ADC HAT)
 * ============================================================ */
#define ADC_MOSI   GPIO10
#define ADC_MISO   GPIO9
#define ADC_SCLK   GPIO11
#define ADC_CS     GPIO8
#define ADC_DRDY   GPIO17
#define ADC_RST    GPIO18

/* ============================================================
 * ADS1263 Registers & Commands
 * ============================================================ */
#define CMD_RESET    0x06
#define CMD_START1   0x08
#define CMD_STOP1    0x0A
#define CMD_RDATA1   0x12
#define CMD_RREG     0x20
#define CMD_WREG     0x40
#define CMD_SFOCAL1  0x19

#define REG_ID        0x00
#define REG_POWER     0x01
#define REG_INTERFACE 0x02
#define REG_MODE0     0x03
#define REG_MODE1     0x04
#define REG_MODE2     0x05
#define REG_INPMUX    0x06
#define REG_OFCAL0    0x07
#define REG_OFCAL1    0x08
#define REG_OFCAL2    0x09
#define REG_REFMUX    0x0F

/* Data rates */
#define DR_38400   0x0F

/* Gains */
#define GAIN_1     0x00
#define GAIN_2     0x10
#define GAIN_4     0x20

/* Reference voltage */
#define VREF       2.5

/* ADS1263 ID check */
#define ADS1263_DEV_ID  0x01

/* ============================================================
 * Vibration Capture Configuration
 * ============================================================ */
#define SAMPLE_RATE      38400     /* Max ADS1263 ADC1 rate */
#define CAPTURE_SECONDS  1         /* Default capture duration */
#define MAX_SAMPLES      (SAMPLE_RATE * 5)  /* Max 5 seconds buffer */
#define DRDY_TIMEOUT_US  100000    /* 100 ms per-sample timeout */
#define FFT_SIZE         4096      /* Power-of-2 FFT size */

/* ============================================================
 * Timestamp helpers
 * ============================================================ */
static inline uint64_t timestamp_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
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
 * SPI Bit-bang (Mode 1: CPOL=0, CPHA=1) — Fast version
 *
 * At 38,400 SPS bit-banged SPI is the bottleneck, so we minimize
 * delays. The mmap pinread/pinwrite at ~10 ns gives us ~6.25 MHz
 * effective SPI clock without any usleep() calls.
 * ============================================================ */

static int spi_init(void)
{
    pin_t p;

    p = pinopen(ADC_SCLK, OUTPUT);
    if (p.mode == UNDEF) return -1;
    pinwrite(ADC_SCLK, LOW);

    p = pinopen(ADC_MOSI, OUTPUT);
    if (p.mode == UNDEF) return -1;
    pinwrite(ADC_MOSI, LOW);

    p = pinopen(ADC_MISO, INPUT);
    if (p.mode == UNDEF) return -1;
    pinpull(ADC_MISO, PULL_NONE);

    p = pinopen(ADC_CS, OUTPUT);
    if (p.mode == UNDEF) return -1;
    pinwrite(ADC_CS, HIGH);

    p = pinopen(ADC_DRDY, INPUT);
    if (p.mode == UNDEF) return -1;
    pinpull(ADC_DRDY, PULL_UP);

    p = pinopen(ADC_RST, OUTPUT);
    if (p.mode == UNDEF) return -1;
    pinwrite(ADC_RST, HIGH);

    return 0;
}

static void spi_cleanup(void)
{
    pinclose(ADC_SCLK);
    pinclose(ADC_MOSI);
    pinclose(ADC_MISO);
    pinclose(ADC_CS);
    pinclose(ADC_DRDY);
    pinclose(ADC_RST);
}

/*
 * Fast SPI byte transfer — NO usleep() delays.
 * Relies on mmap GPIO access latency (~10-50 ns) as the clock period.
 * This gives us approximately 5-10 MHz effective SPI clock.
 */
static uint8_t spi_xfer_fast(uint8_t tx)
{
    uint8_t rx = 0;

    for (int i = 7; i >= 0; i--)
    {
        pinwrite(ADC_SCLK, HIGH);
        pinwrite(ADC_MOSI, (tx >> i) & 1);
        /* No delay — mmap access is ~10 ns, giving ~50 MHz max toggle */
        pinwrite(ADC_SCLK, LOW);
        if (pinread(ADC_MISO))
            rx |= (1 << i);
    }

    return rx;
}

static void spi_write_cmd(uint8_t cmd)
{
    pinwrite(ADC_CS, LOW);
    spi_xfer_fast(cmd);
    pinwrite(ADC_CS, HIGH);
}

static void spi_write_bytes(const uint8_t *tx, int len)
{
    pinwrite(ADC_CS, LOW);
    for (int i = 0; i < len; i++)
        spi_xfer_fast(tx[i]);
    pinwrite(ADC_CS, HIGH);
}

static void spi_transfer(const uint8_t *tx, uint8_t *rx, int len)
{
    pinwrite(ADC_CS, LOW);
    for (int i = 0; i < len; i++)
        rx[i] = spi_xfer_fast(tx[i]);
    pinwrite(ADC_CS, HIGH);
}

/* ============================================================
 * ADS1263 Driver (minimal for vibration capture)
 * ============================================================ */

static uint8_t ads_read_reg(uint8_t reg)
{
    uint8_t tx[3] = { (uint8_t)(CMD_RREG | (reg & 0x1F)), 0x00, 0x00 };
    uint8_t rx[3];
    spi_transfer(tx, rx, 3);
    return rx[2];
}

static void ads_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[3] = { (uint8_t)(CMD_WREG | (reg & 0x1F)), 0x00, val };
    spi_write_bytes(tx, 3);
}

static int ads_wait_drdy(void)
{
    int elapsed = 0;
    while (pinread(ADC_DRDY) != LOW)
    {
        usleep(1);
        elapsed += 1;
        if (elapsed >= DRDY_TIMEOUT_US)
            return -1;
    }
    return 0;
}

/* Fast DRDY wait — busy-poll without usleep() for minimum latency */
static int ads_wait_drdy_fast(void)
{
    uint64_t deadline = timestamp_ns() + (uint64_t)DRDY_TIMEOUT_US * 1000ULL;
    while (pinread(ADC_DRDY) != LOW)
    {
        if (timestamp_ns() > deadline)
            return -1;
    }
    return 0;
}

static void ads_hw_reset(void)
{
    pinwrite(ADC_RST, LOW);
    usleep(200000);
    pinwrite(ADC_RST, HIGH);
    usleep(200000);
}

static int32_t ads_read_adc1_fast(void)
{
    uint8_t tx[6] = { CMD_RDATA1, 0, 0, 0, 0, 0 };
    uint8_t rx[6];
    spi_transfer(tx, rx, 6);

    /* rx[1]=status, rx[2..5]=32-bit data MSB first */
    return ((int32_t)rx[2] << 24) |
           ((int32_t)rx[3] << 16) |
           ((int32_t)rx[4] <<  8) |
           ((int32_t)rx[5]);
}

static double ads_code_to_voltage(int32_t raw, double gain)
{
    return ((double)raw / 2147483648.0) * (VREF / gain);
}

static void ads_set_channel(uint8_t pos, uint8_t neg)
{
    ads_write_reg(REG_INPMUX, ((pos & 0x0F) << 4) | (neg & 0x0F));
}

/* Initialize ADS1263 for maximum-speed vibration capture */
static int ads_init_vibration(void)
{
    printf("[ADS1263] Initializing for vibration capture at %d SPS...\n", SAMPLE_RATE);

    if (spi_init() < 0)
    {
        fprintf(stderr, "[ADS1263] FAIL: SPI init\n");
        return -1;
    }

    ads_hw_reset();

    /* Verify chip ID */
    uint8_t id = ads_read_reg(REG_ID);
    printf("[ADS1263] Chip ID: 0x%02X\n", id);
    if (id == 0x00 || id == 0xFF)
    {
        fprintf(stderr, "[ADS1263] ERROR: No device detected (ID=0x%02X)\n", id);
        return -1;
    }

    /* MODE0: continuous conversion mode */
    ads_write_reg(REG_MODE0, 0x00);

    /* MODE1: FIR filter (low latency for high DR) */
    ads_write_reg(REG_MODE1, 0x00); /* sinc1 = minimum latency */

    /* MODE2: Gain=1, DR=38400 SPS */
    ads_write_reg(REG_MODE2, GAIN_1 | DR_38400);

    /* Internal 2.5V reference */
    ads_write_reg(REG_REFMUX, 0x00);
    ads_write_reg(REG_POWER, 0x01);

    /* No status or CRC for speed */
    ads_write_reg(REG_INTERFACE, 0x00);

    /* Default channel: AIN0 vs AINCOM */
    ads_set_channel(0, 0x0A);

    usleep(50000); /* 50 ms settling */

    /* Self-calibration */
    printf("[ADS1263] Running self-offset calibration...\n");
    spi_write_cmd(CMD_SFOCAL1);
    usleep(500000);

    printf("[ADS1263] Configuration:\n");
    printf("  MODE0    = 0x%02X\n", ads_read_reg(REG_MODE0));
    printf("  MODE1    = 0x%02X\n", ads_read_reg(REG_MODE1));
    printf("  MODE2    = 0x%02X\n", ads_read_reg(REG_MODE2));
    printf("  INPMUX   = 0x%02X\n", ads_read_reg(REG_INPMUX));
    printf("  POWER    = 0x%02X\n", ads_read_reg(REG_POWER));
    printf("  Ready for capture.\n\n");

    return 0;
}

/* ============================================================
 * Vibration Data Buffer
 * ============================================================ */
typedef struct {
    int32_t  *raw;            /* Raw ADC codes */
    uint64_t *timestamps;     /* Nanosecond timestamps per sample */
    int       count;          /* Number of samples captured */
    int       capacity;       /* Buffer size */
    double    actual_rate;    /* Measured sample rate */
    double    duration_sec;   /* Actual capture duration */
} vib_buffer_t;

static vib_buffer_t *vib_alloc(int capacity)
{
    vib_buffer_t *buf = (vib_buffer_t *)calloc(1, sizeof(vib_buffer_t));
    if (!buf) return NULL;

    buf->raw = (int32_t *)malloc(capacity * sizeof(int32_t));
    buf->timestamps = (uint64_t *)malloc(capacity * sizeof(uint64_t));
    buf->capacity = capacity;

    if (!buf->raw || !buf->timestamps)
    {
        free(buf->raw);
        free(buf->timestamps);
        free(buf);
        return NULL;
    }
    return buf;
}

static void vib_free(vib_buffer_t *buf)
{
    if (!buf) return;
    free(buf->raw);
    free(buf->timestamps);
    free(buf);
}

/* ============================================================
 * Test 1: Capture Rate Benchmark
 *
 * Measures how close we get to 38,400 SPS with bit-banged SPI.
 * ============================================================ */
static int test_capture_rate(void)
{
    printf("--- Test 1: Capture Rate Benchmark ---\n");
    printf("  Target: %d SPS (ADS1263 ADC1 max rate)\n\n", SAMPLE_RATE);

    /* Start continuous conversion */
    ads_set_channel(0, 0x0A);
    spi_write_cmd(CMD_START1);
    usleep(1000);

    const int BENCH_SAMPLES = 5000;
    int captured = 0;
    int drdy_timeouts = 0;
    uint64_t t_start = timestamp_ns();

    for (int i = 0; i < BENCH_SAMPLES; i++)
    {
        if (ads_wait_drdy_fast() < 0)
        {
            drdy_timeouts++;
            continue;
        }
        ads_read_adc1_fast();
        captured++;
    }

    uint64_t t_end = timestamp_ns();
    double elapsed_s = (double)(t_end - t_start) / 1.0e9;
    double actual_sps = captured / elapsed_s;

    spi_write_cmd(CMD_STOP1);

    printf("  Results:\n");
    printf("    Requested:       %d samples\n", BENCH_SAMPLES);
    printf("    Captured:        %d samples\n", captured);
    printf("    DRDY timeouts:   %d\n", drdy_timeouts);
    printf("    Elapsed:         %.3f seconds\n", elapsed_s);
    printf("    Actual rate:     %.0f SPS\n", actual_sps);
    printf("    Efficiency:      %.1f%% of target\n", (actual_sps / SAMPLE_RATE) * 100.0);
    printf("    Avg time/sample: %.1f µs\n", elapsed_s * 1.0e6 / captured);
    printf("\n");

    if (actual_sps >= SAMPLE_RATE * 0.95)
        printf("  PASS: Achieving >=95%% of target rate\n");
    else if (actual_sps >= SAMPLE_RATE * 0.5)
        printf("  WARNING: Only %.0f%% of target — SPI overhead limits throughput\n",
               (actual_sps / SAMPLE_RATE) * 100.0);
    else
        printf("  FAIL: Rate too low. Consider hardware SPI (/dev/spidev0.0)\n");

    printf("\n");
    return 0;
}

/* ============================================================
 * Test 2: Single-Channel Vibration Capture + Statistics
 * ============================================================ */
static int test_vibration_capture(int seconds, int save_csv)
{
    int target_samples = SAMPLE_RATE * seconds;
    if (target_samples > MAX_SAMPLES) target_samples = MAX_SAMPLES;

    printf("--- Test 2: Vibration Capture ---\n");
    printf("  Channel: AIN0 vs AINCOM\n");
    printf("  Rate: %d SPS,  Duration: %d sec,  Samples: %d\n",
           SAMPLE_RATE, seconds, target_samples);
    if (save_csv) printf("  Output: vibration_data.csv\n");
    printf("\n");

    vib_buffer_t *buf = vib_alloc(target_samples);
    if (!buf)
    {
        fprintf(stderr, "  FAIL: Could not allocate buffer for %d samples\n", target_samples);
        return -1;
    }

    signal(SIGINT, signal_handler);

    /* Start continuous conversion on AIN0 */
    ads_set_channel(0, 0x0A);
    spi_write_cmd(CMD_START1);
    usleep(1000);

    uint64_t t_start = timestamp_ns();
    int drdy_miss = 0;

    printf("  Capturing... ");
    fflush(stdout);

    for (int i = 0; i < target_samples && g_running; i++)
    {
        if (ads_wait_drdy_fast() < 0)
        {
            drdy_miss++;
            i--; /* Retry */
            if (drdy_miss > target_samples / 10) break;
            continue;
        }

        buf->timestamps[buf->count] = timestamp_ns();
        buf->raw[buf->count] = ads_read_adc1_fast();
        buf->count++;

        /* Progress indicator every 10% */
        if (buf->count % (target_samples / 10) == 0)
        {
            printf("%d%% ", (buf->count * 100) / target_samples);
            fflush(stdout);
        }
    }

    uint64_t t_end = timestamp_ns();
    spi_write_cmd(CMD_STOP1);

    buf->duration_sec = (double)(t_end - t_start) / 1.0e9;
    buf->actual_rate = buf->count / buf->duration_sec;

    printf("\n\n");
    printf("  Capture complete:\n");
    printf("    Samples:       %d\n", buf->count);
    printf("    Duration:      %.3f s\n", buf->duration_sec);
    printf("    Actual rate:   %.0f SPS\n", buf->actual_rate);
    printf("    DRDY misses:   %d\n", drdy_miss);
    printf("\n");

    /* Statistics */
    if (buf->count > 0)
    {
        double sum = 0, min_v = 1e18, max_v = -1e18;
        double gain = 1.0; /* GAIN_1 */

        for (int i = 0; i < buf->count; i++)
        {
            double v = ads_code_to_voltage(buf->raw[i], gain);
            sum += v;
            if (v < min_v) min_v = v;
            if (v > max_v) max_v = v;
        }
        double mean_v = sum / buf->count;

        /* RMS and std deviation */
        double var_sum = 0, rms_sum = 0;
        for (int i = 0; i < buf->count; i++)
        {
            double v = ads_code_to_voltage(buf->raw[i], gain);
            double diff = v - mean_v;
            var_sum += diff * diff;
            rms_sum += v * v;
        }
        double stddev = sqrt(var_sum / buf->count);
        double rms = sqrt(rms_sum / buf->count);
        double vpp = max_v - min_v;

        printf("  Voltage Statistics:\n");
        printf("    Mean:      %+.6f V\n", mean_v);
        printf("    Min:       %+.6f V\n", min_v);
        printf("    Max:       %+.6f V\n", max_v);
        printf("    Vpp:        %.6f V\n", vpp);
        printf("    RMS:        %.6f V\n", rms);
        printf("    AC RMS:     %.6f V (signal only, DC removed)\n", stddev);
        printf("    StdDev:     %.6f V\n", stddev);
        printf("    SNR (est):  %.1f dB\n", 20.0 * log10(mean_v != 0 ? fabs(mean_v) / stddev : 1.0));
        printf("\n");

        /* Timestamp jitter analysis */
        if (buf->count > 1)
        {
            double expected_period_ns = 1.0e9 / SAMPLE_RATE;
            double jit_sum = 0, jit_max = 0;
            for (int i = 1; i < buf->count; i++)
            {
                double dt = (double)(buf->timestamps[i] - buf->timestamps[i - 1]);
                double jitter = fabs(dt - expected_period_ns);
                jit_sum += jitter;
                if (jitter > jit_max) jit_max = jitter;
            }
            double jit_avg = jit_sum / (buf->count - 1);
            printf("  Timing Jitter:\n");
            printf("    Expected period:  %.1f µs\n", expected_period_ns / 1000.0);
            printf("    Avg jitter:       %.1f µs\n", jit_avg / 1000.0);
            printf("    Max jitter:       %.1f µs\n", jit_max / 1000.0);
            printf("\n");
        }
    }

    /* Save to CSV */
    if (save_csv && buf->count > 0)
    {
        FILE *f = fopen("vibration_data.csv", "w");
        if (f)
        {
            fprintf(f, "sample,timestamp_us,raw_code,voltage_V\n");
            for (int i = 0; i < buf->count; i++)
            {
                double v = ads_code_to_voltage(buf->raw[i], 1.0);
                fprintf(f, "%d,%.3f,%d,%.9f\n",
                        i,
                        (double)(buf->timestamps[i] - buf->timestamps[0]) / 1000.0,
                        buf->raw[i], v);
            }
            fclose(f);
            printf("  CSV saved: vibration_data.csv (%d samples)\n\n", buf->count);
        }
        else
        {
            fprintf(stderr, "  WARNING: Could not write vibration_data.csv\n\n");
        }
    }

    vib_free(buf);
    return 0;
}

/* ============================================================
 * Test 3: Basic Spectral Analysis (DFT)
 *
 * Simple magnitude spectrum using a real-valued DFT
 * (no external FFT library required). Sufficient for identifying
 * dominant vibration frequencies.
 *
 * For production use, link with FFTW3 for much better performance.
 * ============================================================ */

/* Hanning window */
static double hanning(int i, int N)
{
    return 0.5 * (1.0 - cos(2.0 * M_PI * i / (N - 1)));
}

/*
 * Real DFT (slow but dependency-free). Computes first N/2 bins.
 * magnitude[k] = |X[k]| for k = 0..N/2-1
 * freq[k] = k * sample_rate / N
 */
static void compute_dft_magnitude(const double *signal, int N,
                                   double *magnitude, double sample_rate)
{
    int half = N / 2;
    for (int k = 0; k < half; k++)
    {
        double re = 0, im = 0;
        for (int n = 0; n < N; n++)
        {
            double angle = 2.0 * M_PI * k * n / N;
            re += signal[n] * cos(angle);
            im -= signal[n] * sin(angle);
        }
        magnitude[k] = sqrt(re * re + im * im) / N;
    }
    (void)sample_rate;
}

static int test_spectral_analysis(void)
{
    int N = FFT_SIZE; /* 4096 samples for spectrum */

    printf("--- Test 3: Spectral Analysis ---\n");
    printf("  Capturing %d samples at %d SPS for frequency analysis...\n", N, SAMPLE_RATE);
    printf("  Frequency resolution: %.2f Hz\n", (double)SAMPLE_RATE / N);
    printf("  Max frequency: %d Hz (Nyquist)\n\n", SAMPLE_RATE / 2);

    vib_buffer_t *buf = vib_alloc(N);
    if (!buf)
    {
        fprintf(stderr, "  FAIL: Memory allocation\n");
        return -1;
    }

    signal(SIGINT, signal_handler);

    /* Capture N samples */
    ads_set_channel(0, 0x0A);
    spi_write_cmd(CMD_START1);
    usleep(1000);

    for (int i = 0; i < N && g_running; i++)
    {
        if (ads_wait_drdy_fast() < 0) { i--; continue; }
        buf->raw[buf->count] = ads_read_adc1_fast();
        buf->count++;
    }
    spi_write_cmd(CMD_STOP1);

    if (buf->count < N)
    {
        fprintf(stderr, "  Not enough samples (%d/%d)\n", buf->count, N);
        vib_free(buf);
        return -1;
    }

    printf("  Captured %d samples. Computing DFT...\n", buf->count);
    printf("  (This may take a few seconds for N=%d)\n\n", N);

    /* Convert to voltage and apply window */
    double *windowed = (double *)malloc(N * sizeof(double));
    double *magnitude = (double *)malloc((N / 2) * sizeof(double));

    if (!windowed || !magnitude)
    {
        free(windowed);
        free(magnitude);
        vib_free(buf);
        return -1;
    }

    double dc_sum = 0;
    for (int i = 0; i < N; i++)
    {
        double v = ads_code_to_voltage(buf->raw[i], 1.0);
        dc_sum += v;
    }
    double dc_offset = dc_sum / N;

    for (int i = 0; i < N; i++)
    {
        double v = ads_code_to_voltage(buf->raw[i], 1.0);
        windowed[i] = (v - dc_offset) * hanning(i, N); /* Remove DC, apply window */
    }

    compute_dft_magnitude(windowed, N, magnitude, SAMPLE_RATE);

    /* Find top 5 peaks */
    double freq_res = (double)SAMPLE_RATE / N;

    typedef struct { int bin; double mag; double freq; } peak_t;
    peak_t peaks[5] = {};

    for (int k = 1; k < N / 2; k++) /* skip DC bin */
    {
        /* Find minimum in top peaks */
        int min_idx = 0;
        for (int p = 1; p < 5; p++)
            if (peaks[p].mag < peaks[min_idx].mag)
                min_idx = p;

        if (magnitude[k] > peaks[min_idx].mag)
        {
            peaks[min_idx].bin = k;
            peaks[min_idx].mag = magnitude[k];
            peaks[min_idx].freq = k * freq_res;
        }
    }

    /* Sort peaks by magnitude (descending) */
    for (int i = 0; i < 4; i++)
        for (int j = i + 1; j < 5; j++)
            if (peaks[j].mag > peaks[i].mag)
            {
                peak_t tmp = peaks[i];
                peaks[i] = peaks[j];
                peaks[j] = tmp;
            }

    printf("  DC offset removed: %.6f V\n\n", dc_offset);
    printf("  Top 5 Frequency Peaks:\n");
    printf("    %-6s  %-14s  %-14s\n", "Rank", "Frequency (Hz)", "Magnitude (V)");
    printf("    %-6s  %-14s  %-14s\n", "----", "--------------", "-------------");

    for (int i = 0; i < 5; i++)
    {
        if (peaks[i].mag > 0)
            printf("    #%-5d  %10.2f Hz    %.9f\n",
                   i + 1, peaks[i].freq, peaks[i].mag);
    }

    printf("\n  Frequency Spectrum (dB, top 20 bins by magnitude):\n");
    printf("    %-14s  %-10s  %s\n", "Frequency (Hz)", "dB", "Bar");
    printf("    %-14s  %-10s  %s\n", "--------------", "----", "---");

    /* Find max for normalization */
    double max_mag = 0;
    for (int k = 1; k < N / 2; k++)
        if (magnitude[k] > max_mag) max_mag = magnitude[k];

    /* Print top bins sorted by frequency (only bins above -60dB) */
    double db_threshold = -60.0;
    int printed = 0;
    for (int k = 1; k < N / 2 && printed < 20; k++)
    {
        if (magnitude[k] <= 0) continue;
        double db = 20.0 * log10(magnitude[k] / max_mag);
        if (db < db_threshold) continue;

        int bar_len = (int)((db + 60.0) / 60.0 * 40);
        if (bar_len < 0) bar_len = 0;
        if (bar_len > 40) bar_len = 40;

        char bar[41];
        memset(bar, '#', bar_len);
        bar[bar_len] = '\0';

        printf("    %10.1f Hz    %+6.1f dB  %s\n", k * freq_res, db, bar);
        printed++;
    }

    /* Save spectrum to CSV */
    FILE *f = fopen("vibration_spectrum.csv", "w");
    if (f)
    {
        fprintf(f, "bin,frequency_Hz,magnitude_V,magnitude_dB\n");
        for (int k = 0; k < N / 2; k++)
        {
            double db = (max_mag > 0 && magnitude[k] > 0) ?
                        20.0 * log10(magnitude[k] / max_mag) : -120.0;
            fprintf(f, "%d,%.4f,%.12f,%.2f\n",
                    k, k * freq_res, magnitude[k], db);
        }
        fclose(f);
        printf("\n  Spectrum saved: vibration_spectrum.csv\n");
    }

    free(windowed);
    free(magnitude);
    vib_free(buf);
    printf("\n");
    return 0;
}

/* ============================================================
 * Test 4: Dual-Channel Interleaved Capture (AIN0 + AIN1)
 * ============================================================ */
static int test_dual_channel(void)
{
    printf("--- Test 4: Dual-Channel Capture (AIN0 + AIN1) ---\n");
    printf("  Interleaved: switch MUX between channels.\n");
    printf("  Effective rate per channel: ~%d SPS each\n\n", SAMPLE_RATE / 2);

    int samples_per_ch = 5000;
    int32_t *ch0_raw = (int32_t *)malloc(samples_per_ch * sizeof(int32_t));
    int32_t *ch1_raw = (int32_t *)malloc(samples_per_ch * sizeof(int32_t));

    if (!ch0_raw || !ch1_raw)
    {
        free(ch0_raw);
        free(ch1_raw);
        return -1;
    }

    signal(SIGINT, signal_handler);

    int ch0_count = 0, ch1_count = 0;
    uint64_t t_start = timestamp_ns();

    for (int i = 0; i < samples_per_ch && g_running; i++)
    {
        /* Channel 0 */
        ads_set_channel(0, 0x0A);
        spi_write_cmd(CMD_START1);
        if (ads_wait_drdy_fast() == 0)
            ch0_raw[ch0_count++] = ads_read_adc1_fast();
        spi_write_cmd(CMD_STOP1);

        /* Channel 1 */
        ads_set_channel(1, 0x0A);
        spi_write_cmd(CMD_START1);
        if (ads_wait_drdy_fast() == 0)
            ch1_raw[ch1_count++] = ads_read_adc1_fast();
        spi_write_cmd(CMD_STOP1);
    }

    uint64_t t_end = timestamp_ns();
    double elapsed_s = (double)(t_end - t_start) / 1.0e9;

    printf("  Capture complete:\n");
    printf("    CH0 samples: %d\n", ch0_count);
    printf("    CH1 samples: %d\n", ch1_count);
    printf("    Duration:    %.3f s\n", elapsed_s);
    printf("    Rate/ch:     %.0f SPS\n", ch0_count / elapsed_s);
    printf("\n");

    /* Basic statistics per channel */
    for (int ch = 0; ch < 2; ch++)
    {
        int32_t *data = (ch == 0) ? ch0_raw : ch1_raw;
        int count = (ch == 0) ? ch0_count : ch1_count;

        if (count == 0) continue;

        double sum = 0, min_v = 1e18, max_v = -1e18;
        for (int i = 0; i < count; i++)
        {
            double v = ads_code_to_voltage(data[i], 1.0);
            sum += v;
            if (v < min_v) min_v = v;
            if (v > max_v) max_v = v;
        }
        double mean_v = sum / count;
        double var_sum = 0;
        for (int i = 0; i < count; i++)
        {
            double v = ads_code_to_voltage(data[i], 1.0);
            var_sum += (v - mean_v) * (v - mean_v);
        }
        double stddev = sqrt(var_sum / count);

        printf("  AIN%d Statistics:\n", ch);
        printf("    Mean: %+.6f V,  Vpp: %.6f V,  AC RMS: %.6f V\n",
               mean_v, max_v - min_v, stddev);
    }

    free(ch0_raw);
    free(ch1_raw);
    printf("\n");
    return 0;
}

/* ============================================================ */

static void print_usage(const char *prog)
{
    printf("Usage: sudo %s [options]\n", prog);
    printf("Options:\n");
    printf("  (none)    Capture 1 second, print stats\n");
    printf("  rate      Benchmark actual capture rate\n");
    printf("  fft       Capture + spectral analysis\n");
    printf("  log       Capture + save CSV\n");
    printf("  dual      Dual-channel interleaved\n");
    printf("  all       Run all tests\n");
    printf("  help      Show this message\n");
}

int main(int argc, char *argv[])
{
    int result = 0;
    int do_rate  = 0;
    int do_capture = 0;
    int do_fft   = 0;
    int do_csv   = 0;
    int do_dual  = 0;

    if (argc < 2) do_capture = 1;

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "rate") == 0)         do_rate = 1;
        else if (strcmp(argv[i], "fft") == 0)   { do_capture = 1; do_fft = 1; }
        else if (strcmp(argv[i], "log") == 0)   { do_capture = 1; do_csv = 1; }
        else if (strcmp(argv[i], "dual") == 0)    do_dual = 1;
        else if (strcmp(argv[i], "all") == 0)   { do_rate = 1; do_capture = 1; do_fft = 1; do_dual = 1; }
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

    printf("================================================\n");
    printf(" Vibration Monitoring — ADS1263 at %d SPS\n", SAMPLE_RATE);
    printf(" GPIO_RPI5 Bit-Banged SPI (fast, no delays)\n");
    printf("================================================\n\n");

    if (gpio_init() < 0)
    {
        fprintf(stderr, "FATAL: gpio_init() failed. Are you root on a Raspberry Pi 5?\n");
        return 1;
    }

    if (ads_init_vibration() < 0)
    {
        fprintf(stderr, "FATAL: ADS1263 init failed\n");
        spi_cleanup();
        gpio_cleanup();
        return 1;
    }

    /* Test 1: Rate benchmark */
    if (do_rate)
    {
        if (test_capture_rate() < 0)
            result = 1;
    }

    /* Test 2: Vibration capture */
    if (do_capture)
    {
        if (test_vibration_capture(CAPTURE_SECONDS, do_csv) < 0)
            result = 1;
    }

    /* Test 3: Spectral analysis */
    if (do_fft)
    {
        if (test_spectral_analysis() < 0)
            result = 1;
    }

    /* Test 4: Dual channel */
    if (do_dual)
    {
        if (test_dual_channel() < 0)
            result = 1;
    }

    spi_cleanup();
    gpio_cleanup();

    printf("=== %s ===\n", result ? "SOME TESTS FAILED" : "ALL TESTS PASSED");
    return result;
}
