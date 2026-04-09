/*
 * test_turbine.cpp - Turbine Monitoring Framework
 *
 * Combined blade detection + vibration monitoring framework with
 * an abstraction layer for future fast SAR ADC integration.
 *
 * Architecture:
 *   ┌────────────────────────────────────────────────┐
 *   │  Turbine Monitor                               │
 *   ├────────────────────────────────────────────────┤
 *   │  Blade Detector (GPIO digital, ns timestamps)  │
 *   │  Vibration ADC  (abstract interface)           │
 *   │    ├── ADS1263 backend  (current, 38.4k SPS)   │
 *   │    └── AD7606 backend   (200k SPS, 8ch simult.) │
 *   │  Data Logger (CSV output)                      │
 *   │  Alert Monitor (threshold-based)               │
 *   └────────────────────────────────────────────────┘
 *
 * Compile:
 *   g++ test_turbine.cpp -I.. -L.. -l:gpio_rpi5.a -o test_turbine -Wall -Wextra -O2 -lm
 *
 * Run:
 *   sudo ./test_turbine              # monitor for 30 seconds
 *   sudo ./test_turbine 60           # monitor for 60 seconds
 *   sudo ./test_turbine sim          # simulate signals (no hardware needed)
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
 * Pin Definitions
 * ============================================================ */
/* Blade detection */
#define PIN_BLADE_SENSOR  GPIO22   /* Hall/inductive sensor (active LOW) */
#define PIN_INDEX_SENSOR  GPIO23   /* 1x/rev index pulse (optional) */

/* ADS1263 SPI (Waveshare HAT) */
#define PIN_SPI_MOSI   GPIO10
#define PIN_SPI_MISO   GPIO9
#define PIN_SPI_SCLK   GPIO11
#define PIN_SPI_CS0    GPIO8     /* ADS1263 CS */
#define PIN_ADC_DRDY   GPIO17
#define PIN_ADC_RST    GPIO18

/* Future SAR ADC (can share SPI bus or use SPI1) */
#define PIN_SPI1_MOSI  GPIO20    /* SPI1 for future SAR ADC */
#define PIN_SPI1_MISO  GPIO19
#define PIN_SPI1_SCLK  GPIO21
#define PIN_SPI1_CS0   GPIO16    /* SAR ADC CS */
#define PIN_SAR_DRDY   GPIO24    /* SAR ADC data ready */

/* Alert / status LEDs (optional) */
#define PIN_LED_STATUS GPIO25    /* Status LED */
#define PIN_LED_ALERT  GPIO26    /* Over-speed / vibration alert */

/* ============================================================
 * Configuration
 * ============================================================ */
#define NUM_BLADES           10
#define MAX_RPM              400000
#define RPM_DEBOUNCE_NS      2000
#define RPM_WINDOW           32
#define VIB_SAMPLE_RATE      38400
#define VIB_BUFFER_SIZE      4096
#define MONITOR_INTERVAL_MS  100    /* Dashboard update interval */

/* Alert thresholds */
#define ALERT_RPM_HIGH       380000.0   /* Over-speed warning */
#define ALERT_RPM_LOW        100.0      /* Stall detection */
#define ALERT_VIB_RMS_V      0.5        /* Excessive vibration */
#define ALERT_JITTER_PCT     5.0        /* Blade timing jitter */

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
 * ADC Backend Abstraction
 *
 * This interface allows swapping between ADS1263 (current) and
 * a future fast SAR ADC (e.g., ADS127L11, AD7606) without
 * changing the monitoring logic.
 * ============================================================ */
typedef struct adc_backend adc_backend_t;

struct adc_backend {
    const char *name;
    int         max_sps;        /* Maximum sample rate */
    int         resolution;     /* Bits */
    double      vref;

    /* Function pointers — implement per backend */
    int     (*init)(adc_backend_t *self);
    void    (*cleanup)(adc_backend_t *self);
    int     (*set_channel)(adc_backend_t *self, int channel);
    int     (*start)(adc_backend_t *self);
    void    (*stop)(adc_backend_t *self);
    int     (*wait_ready)(adc_backend_t *self);
    int32_t (*read_raw)(adc_backend_t *self);
    double  (*to_voltage)(adc_backend_t *self, int32_t raw);

    /* Backend-specific data */
    void   *priv;
};

/* ============================================================
 * ADS1263 Backend Implementation
 * ============================================================ */

/* ADS1263 registers/commands (minimal set) */
#define ADS_CMD_RESET   0x06
#define ADS_CMD_START1  0x08
#define ADS_CMD_STOP1   0x0A
#define ADS_CMD_RDATA1  0x12
#define ADS_CMD_RREG    0x20
#define ADS_CMD_WREG    0x40
#define ADS_CMD_SFOCAL1 0x19

#define ADS_REG_ID      0x00
#define ADS_REG_POWER   0x01
#define ADS_REG_IFACE   0x02
#define ADS_REG_MODE0   0x03
#define ADS_REG_MODE1   0x04
#define ADS_REG_MODE2   0x05
#define ADS_REG_INPMUX  0x06
#define ADS_REG_REFMUX  0x0F

#define ADS_DR_38400    0x0F
#define ADS_GAIN_1      0x00

/* Fast SPI byte transfer (no delays, mmap-speed) */
static uint8_t ads_spi_xfer(uint8_t tx)
{
    uint8_t rx = 0;
    for (int i = 7; i >= 0; i--)
    {
        pinwrite(PIN_SPI_SCLK, HIGH);
        pinwrite(PIN_SPI_MOSI, (tx >> i) & 1);
        pinwrite(PIN_SPI_SCLK, LOW);
        if (pinread(PIN_SPI_MISO))
            rx |= (1 << i);
    }
    return rx;
}

static void ads_spi_cmd(uint8_t cmd)
{
    pinwrite(PIN_SPI_CS0, LOW);
    ads_spi_xfer(cmd);
    pinwrite(PIN_SPI_CS0, HIGH);
}

static uint8_t ads_reg_read(uint8_t reg)
{
    uint8_t rx;
    pinwrite(PIN_SPI_CS0, LOW);
    ads_spi_xfer(ADS_CMD_RREG | (reg & 0x1F));
    ads_spi_xfer(0x00);
    rx = ads_spi_xfer(0x00);
    pinwrite(PIN_SPI_CS0, HIGH);
    return rx;
}

static void ads_reg_write(uint8_t reg, uint8_t val)
{
    pinwrite(PIN_SPI_CS0, LOW);
    ads_spi_xfer(ADS_CMD_WREG | (reg & 0x1F));
    ads_spi_xfer(0x00);
    ads_spi_xfer(val);
    pinwrite(PIN_SPI_CS0, HIGH);
}

static int ads1263_init(adc_backend_t *self)
{
    (void)self;
    pin_t p;

    p = pinopen(PIN_SPI_SCLK, OUTPUT); if (p.mode == UNDEF) return -1;
    pinwrite(PIN_SPI_SCLK, LOW);
    p = pinopen(PIN_SPI_MOSI, OUTPUT); if (p.mode == UNDEF) return -1;
    p = pinopen(PIN_SPI_MISO, INPUT);  if (p.mode == UNDEF) return -1;
    pinpull(PIN_SPI_MISO, PULL_NONE);
    p = pinopen(PIN_SPI_CS0, OUTPUT);  if (p.mode == UNDEF) return -1;
    pinwrite(PIN_SPI_CS0, HIGH);
    p = pinopen(PIN_ADC_DRDY, INPUT);  if (p.mode == UNDEF) return -1;
    pinpull(PIN_ADC_DRDY, PULL_UP);
    p = pinopen(PIN_ADC_RST, OUTPUT);  if (p.mode == UNDEF) return -1;
    pinwrite(PIN_ADC_RST, HIGH);

    /* Hardware reset */
    pinwrite(PIN_ADC_RST, LOW);
    usleep(200000);
    pinwrite(PIN_ADC_RST, HIGH);
    usleep(200000);

    /* Verify chip */
    uint8_t id = ads_reg_read(ADS_REG_ID);
    if (id == 0x00 || id == 0xFF) return -1;

    /* Configure for max speed vibration */
    ads_reg_write(ADS_REG_MODE0, 0x00);
    ads_reg_write(ADS_REG_MODE1, 0x00);           /* sinc1, min latency */
    ads_reg_write(ADS_REG_MODE2, ADS_GAIN_1 | ADS_DR_38400);
    ads_reg_write(ADS_REG_REFMUX, 0x00);
    ads_reg_write(ADS_REG_POWER, 0x01);
    ads_reg_write(ADS_REG_IFACE, 0x00);           /* no status, no CRC */
    ads_reg_write(ADS_REG_INPMUX, 0x0A);          /* AIN0 vs AINCOM */

    usleep(50000);
    ads_spi_cmd(ADS_CMD_SFOCAL1);
    usleep(500000);

    return 0;
}

static void ads1263_cleanup(adc_backend_t *self)
{
    (void)self;
    ads_spi_cmd(ADS_CMD_STOP1);
    pinclose(PIN_SPI_SCLK);
    pinclose(PIN_SPI_MOSI);
    pinclose(PIN_SPI_MISO);
    pinclose(PIN_SPI_CS0);
    pinclose(PIN_ADC_DRDY);
    pinclose(PIN_ADC_RST);
}

static int ads1263_set_channel(adc_backend_t *self, int ch)
{
    (void)self;
    ads_reg_write(ADS_REG_INPMUX, ((ch & 0x0F) << 4) | 0x0A);
    return 0;
}

static int ads1263_start(adc_backend_t *self)
{
    (void)self;
    ads_spi_cmd(ADS_CMD_START1);
    return 0;
}

static void ads1263_stop(adc_backend_t *self)
{
    (void)self;
    ads_spi_cmd(ADS_CMD_STOP1);
}

static int ads1263_wait_ready(adc_backend_t *self)
{
    (void)self;
    uint64_t deadline = time_ns() + 100000000ULL; /* 100 ms */
    while (pinread(PIN_ADC_DRDY) != LOW)
        if (time_ns() > deadline) return -1;
    return 0;
}

static int32_t ads1263_read_raw(adc_backend_t *self)
{
    (void)self;
    pinwrite(PIN_SPI_CS0, LOW);
    ads_spi_xfer(ADS_CMD_RDATA1);
    uint8_t s = ads_spi_xfer(0); (void)s;
    uint8_t b0 = ads_spi_xfer(0);
    uint8_t b1 = ads_spi_xfer(0);
    uint8_t b2 = ads_spi_xfer(0);
    uint8_t b3 = ads_spi_xfer(0);
    pinwrite(PIN_SPI_CS0, HIGH);
    return ((int32_t)b0 << 24) | ((int32_t)b1 << 16) | ((int32_t)b2 << 8) | b3;
}

static double ads1263_to_voltage(adc_backend_t *self, int32_t raw)
{
    return ((double)raw / 2147483648.0) * self->vref;
}

static adc_backend_t g_ads1263 = {
    .name       = "ADS1263 (32-bit Delta-Sigma)",
    .max_sps    = 38400,
    .resolution = 32,
    .vref       = 2.5,
    .init       = ads1263_init,
    .cleanup    = ads1263_cleanup,
    .set_channel = ads1263_set_channel,
    .start      = ads1263_start,
    .stop       = ads1263_stop,
    .wait_ready = ads1263_wait_ready,
    .read_raw   = ads1263_read_raw,
    .to_voltage = ads1263_to_voltage,
    .priv       = NULL
};

/* ============================================================
 * AD7606 SAR ADC Backend (16-bit, 200 kSPS, 8ch simultaneous)
 *
 * Protocol: CONVST pulse → wait BUSY LOW → SPI read 8×16-bit
 * Uses SPI1 pins so both ADCs can coexist.
 *
 * AD7606 always reads all 8 channels simultaneously.
 * set_channel() selects which channel to return via read_raw().
 * ============================================================ */
#define AD7606_CONVST  GPIO27   /* Convert start (rising edge) */
#define AD7606_RANGE   GPIO5    /* LOW=±5V, HIGH=±10V */
#define AD7606_NUM_CH  8
#define AD7606_FS      5.0      /* ±5V default range */

/* Per-instance data for the AD7606 backend */
typedef struct {
    int      active_ch;                  /* Channel to return from read_raw() */
    int16_t  ch_data[AD7606_NUM_CH];     /* Last conversion results (all 8ch) */
} ad7606_priv_t;

static ad7606_priv_t g_ad7606_priv = {};

/* Fast SPI read 16-bit from AD7606 (no MOSI needed) */
static int16_t ad7606_spi_read16(void)
{
    uint16_t val = 0;
    for (int i = 15; i >= 0; i--)
    {
        pinwrite(PIN_SPI1_SCLK, HIGH);
        pinwrite(PIN_SPI1_SCLK, LOW);
        if (pinread(PIN_SPI1_MISO))
            val |= (1U << i);
    }
    return (int16_t)val;
}

static int ad7606_init(adc_backend_t *self)
{
    pin_t p;

    /* SPI1 pins */
    p = pinopen(PIN_SPI1_SCLK, OUTPUT); if (p.mode == UNDEF) return -1;
    pinwrite(PIN_SPI1_SCLK, LOW);
    p = pinopen(PIN_SPI1_MISO, INPUT);  if (p.mode == UNDEF) return -1;
    pinpull(PIN_SPI1_MISO, PULL_NONE);
    p = pinopen(PIN_SPI1_CS0, OUTPUT);  if (p.mode == UNDEF) return -1;
    pinwrite(PIN_SPI1_CS0, HIGH);

    /* Control pins */
    p = pinopen(AD7606_CONVST, OUTPUT); if (p.mode == UNDEF) return -1;
    pinwrite(AD7606_CONVST, LOW);
    p = pinopen(PIN_SAR_DRDY, INPUT);   if (p.mode == UNDEF) return -1;
    pinpull(PIN_SAR_DRDY, PULL_DOWN);   /* BUSY: HIGH during conversion */
    p = pinopen(AD7606_RANGE, OUTPUT);  if (p.mode == UNDEF) return -1;
    pinwrite(AD7606_RANGE, LOW);        /* ±5V range */

    /* Hardware reset via SAR_DRDY momentarily repurposed — or use a separate pin */
    /* For now, skip reset (board powers up in known state) */
    usleep(1000);

    /* Dummy conversion to clear pipeline */
    pinwrite(AD7606_CONVST, HIGH);
    pinwrite(AD7606_CONVST, LOW);
    usleep(10);
    uint64_t deadline = time_ns() + 100000000ULL;
    while (pinread(PIN_SAR_DRDY) == HIGH)
        if (time_ns() > deadline) return -1;
    pinwrite(PIN_SPI1_CS0, LOW);
    for (int ch = 0; ch < AD7606_NUM_CH; ch++)
        ad7606_spi_read16();
    pinwrite(PIN_SPI1_CS0, HIGH);

    ad7606_priv_t *priv = (ad7606_priv_t *)self->priv;
    priv->active_ch = 0;
    printf("[AD7606] Initialized on SPI1. Range=±%.0fV, 8ch simultaneous.\n", AD7606_FS);
    return 0;
}

static void ad7606_cleanup(adc_backend_t *self)
{
    (void)self;
    pinclose(PIN_SPI1_SCLK);
    pinclose(PIN_SPI1_MISO);
    pinclose(PIN_SPI1_CS0);
    pinclose(AD7606_CONVST);
    pinclose(PIN_SAR_DRDY);
    pinclose(AD7606_RANGE);
}

static int ad7606_set_channel(adc_backend_t *self, int ch)
{
    ad7606_priv_t *priv = (ad7606_priv_t *)self->priv;
    if (ch < 0 || ch >= AD7606_NUM_CH) return -1;
    priv->active_ch = ch;
    return 0;
}

static int ad7606_start(adc_backend_t *self)
{
    (void)self;
    /* AD7606 doesn't have a continuous mode — each CONVST triggers one conversion */
    return 0;
}

static void ad7606_stop(adc_backend_t *self) { (void)self; }

static int ad7606_wait_ready(adc_backend_t *self)
{
    (void)self;
    /* Trigger a conversion */
    pinwrite(AD7606_CONVST, HIGH);
    pinwrite(AD7606_CONVST, LOW);

    /* Wait BUSY LOW (~3.5 µs) */
    uint64_t deadline = time_ns() + 100000000ULL;
    while (pinread(PIN_SAR_DRDY) == HIGH)
        if (time_ns() > deadline) return -1;

    /* Read all 8 channels into priv buffer */
    ad7606_priv_t *priv = (ad7606_priv_t *)self->priv;
    pinwrite(PIN_SPI1_CS0, LOW);
    for (int ch = 0; ch < AD7606_NUM_CH; ch++)
        priv->ch_data[ch] = ad7606_spi_read16();
    pinwrite(PIN_SPI1_CS0, HIGH);

    return 0;
}

static int32_t ad7606_read_raw(adc_backend_t *self)
{
    ad7606_priv_t *priv = (ad7606_priv_t *)self->priv;
    return (int32_t)priv->ch_data[priv->active_ch];
}

static double ad7606_to_voltage(adc_backend_t *self, int32_t raw)
{
    (void)self;
    /* AD7606: 16-bit signed, ±FS. Voltage = raw × FS / 32768 */
    return (double)(int16_t)raw * AD7606_FS / 32768.0;
}

static adc_backend_t g_sar_adc = {
    .name       = "AD7606 (16-bit SAR, 8ch simultaneous)",
    .max_sps    = 200000,
    .resolution = 16,
    .vref       = 2.5,
    .init       = ad7606_init,
    .cleanup    = ad7606_cleanup,
    .set_channel = ad7606_set_channel,
    .start      = ad7606_start,
    .stop       = ad7606_stop,
    .wait_ready = ad7606_wait_ready,
    .read_raw   = ad7606_read_raw,
    .to_voltage = ad7606_to_voltage,
    .priv       = &g_ad7606_priv
};

/* Active ADC backend — change this to switch implementations */
static adc_backend_t *g_active_vib_adc = &g_ads1263;

/* ============================================================
 * Blade Detection Module
 * ============================================================ */
typedef struct {
    /* State */
    uint64_t last_edge_ns;
    int      prev_val;
    uint32_t blade_count;
    uint32_t rev_count;

    /* RPM filter */
    double   periods[RPM_WINDOW];
    int      filt_head;
    int      filt_count;
    double   filt_sum;

    /* Statistics */
    double   rpm_instant;
    double   rpm_avg;
    double   blade_freq;
    double   max_jitter_pct;
} blade_state_t;

static void blade_init(blade_state_t *s)
{
    memset(s, 0, sizeof(*s));
    s->prev_val = HIGH;
}

/* Process one GPIO sample. Returns 1 if blade event detected. */
static int blade_process(blade_state_t *s, int gpio_val, uint64_t now)
{
    int event = 0;

    /* Detect falling edge */
    if (s->prev_val == HIGH && gpio_val == LOW)
    {
        if (s->last_edge_ns > 0 && (now - s->last_edge_ns) >= RPM_DEBOUNCE_NS)
        {
            uint64_t period = now - s->last_edge_ns;
            double period_d = (double)period;

            s->blade_count++;
            s->rev_count = s->blade_count / NUM_BLADES;

            /* Update rolling average */
            if (s->filt_count >= RPM_WINDOW)
                s->filt_sum -= s->periods[s->filt_head];
            else
                s->filt_count++;

            s->periods[s->filt_head] = period_d;
            s->filt_sum += period_d;
            s->filt_head = (s->filt_head + 1) % RPM_WINDOW;

            /* Compute RPM */
            s->rpm_instant = 60.0e9 / (period_d * NUM_BLADES);
            double avg_period = s->filt_sum / s->filt_count;
            s->rpm_avg = 60.0e9 / (avg_period * NUM_BLADES);
            s->blade_freq = 1.0e9 / avg_period;

            /* Jitter (deviation from average) */
            double jitter_pct = fabs(period_d - avg_period) / avg_period * 100.0;
            if (jitter_pct > s->max_jitter_pct)
                s->max_jitter_pct = jitter_pct;

            event = 1;
        }
        s->last_edge_ns = now;
    }
    s->prev_val = gpio_val;
    return event;
}

/* ============================================================
 * Vibration Module
 * ============================================================ */
typedef struct {
    /* Ring buffer of recent samples */
    double   buffer[VIB_BUFFER_SIZE];
    int      head;
    int      count;

    /* Running statistics */
    double   sum;
    double   sum_sq;
    int      total_samples;
    double   min_v, max_v;

    /* Derived */
    double   rms_ac;      /* AC RMS (vibration level) */
    double   mean_v;
    double   vpp;
    double   actual_sps;
} vib_state_t;

static void vib_init(vib_state_t *s)
{
    memset(s, 0, sizeof(*s));
    s->min_v = 1e18;
    s->max_v = -1e18;
}

static void vib_add_sample(vib_state_t *s, double voltage)
{
    /* Ring buffer */
    if (s->count >= VIB_BUFFER_SIZE)
    {
        double old = s->buffer[s->head];
        s->sum -= old;
        s->sum_sq -= old * old;
    }
    else
    {
        s->count++;
    }

    s->buffer[s->head] = voltage;
    s->sum += voltage;
    s->sum_sq += voltage * voltage;
    s->head = (s->head + 1) % VIB_BUFFER_SIZE;
    s->total_samples++;

    if (voltage < s->min_v) s->min_v = voltage;
    if (voltage > s->max_v) s->max_v = voltage;

    /* Update derived stats */
    if (s->count > 0)
    {
        s->mean_v = s->sum / s->count;
        double mean_sq = s->sum_sq / s->count;
        double variance = mean_sq - s->mean_v * s->mean_v;
        s->rms_ac = (variance > 0) ? sqrt(variance) : 0;
        s->vpp = s->max_v - s->min_v;
    }
}

/* ============================================================
 * Alert Monitor
 * ============================================================ */
typedef struct {
    int  overspeed;
    int  stall;
    int  high_vibration;
    int  high_jitter;
    int  alert_count;
    char last_msg[128];
} alert_state_t;

static void alert_init(alert_state_t *a)
{
    memset(a, 0, sizeof(*a));
}

static void alert_check(alert_state_t *a, const blade_state_t *b, const vib_state_t *v)
{
    int prev_count = a->alert_count;

    /* Over-speed */
    if (b->rpm_avg > ALERT_RPM_HIGH && !a->overspeed)
    {
        a->overspeed = 1;
        a->alert_count++;
        snprintf(a->last_msg, sizeof(a->last_msg),
                 "OVERSPEED: %.0f RPM > %.0f limit", b->rpm_avg, ALERT_RPM_HIGH);
    }
    else if (b->rpm_avg <= ALERT_RPM_HIGH * 0.95)
    {
        a->overspeed = 0;
    }

    /* Stall */
    if (b->blade_count > 0 && b->rpm_avg < ALERT_RPM_LOW && b->rpm_avg > 0 && !a->stall)
    {
        a->stall = 1;
        a->alert_count++;
        snprintf(a->last_msg, sizeof(a->last_msg),
                 "STALL: %.0f RPM < %.0f minimum", b->rpm_avg, ALERT_RPM_LOW);
    }
    else if (b->rpm_avg >= ALERT_RPM_LOW)
    {
        a->stall = 0;
    }

    /* Vibration */
    if (v->rms_ac > ALERT_VIB_RMS_V && !a->high_vibration)
    {
        a->high_vibration = 1;
        a->alert_count++;
        snprintf(a->last_msg, sizeof(a->last_msg),
                 "HIGH VIBRATION: AC RMS %.4f V > %.4f limit", v->rms_ac, ALERT_VIB_RMS_V);
    }
    else if (v->rms_ac <= ALERT_VIB_RMS_V * 0.9)
    {
        a->high_vibration = 0;
    }

    /* Jitter */
    if (b->max_jitter_pct > ALERT_JITTER_PCT && !a->high_jitter)
    {
        a->high_jitter = 1;
        a->alert_count++;
        snprintf(a->last_msg, sizeof(a->last_msg),
                 "BLADE JITTER: %.2f%% > %.2f%% limit", b->max_jitter_pct, ALERT_JITTER_PCT);
    }

    if (a->alert_count > prev_count)
        printf("\n  *** ALERT: %s ***\n", a->last_msg);
}

/* ============================================================
 * Data Logger
 * ============================================================ */
typedef struct {
    FILE    *blade_csv;
    FILE    *vib_csv;
    int      blade_entries;
    int      vib_entries;
} logger_t;

static int logger_init(logger_t *log)
{
    memset(log, 0, sizeof(*log));

    log->blade_csv = fopen("turbine_blade_log.csv", "w");
    if (log->blade_csv)
        fprintf(log->blade_csv, "time_us,blade_count,rev,rpm_inst,rpm_avg,blade_freq_Hz\n");

    log->vib_csv = fopen("turbine_vib_log.csv", "w");
    if (log->vib_csv)
        fprintf(log->vib_csv, "time_us,sample,raw,voltage_V\n");

    return (log->blade_csv && log->vib_csv) ? 0 : -1;
}

static void logger_blade_event(logger_t *log, uint64_t t0, const blade_state_t *b)
{
    if (!log->blade_csv) return;
    uint64_t now = time_ns();
    fprintf(log->blade_csv, "%.3f,%u,%u,%.0f,%.0f,%.1f\n",
            (double)(now - t0) / 1000.0,
            b->blade_count, b->rev_count,
            b->rpm_instant, b->rpm_avg, b->blade_freq);
    log->blade_entries++;
}

static void logger_vib_sample(logger_t *log, uint64_t t0, int sample_num,
                               int32_t raw, double voltage)
{
    if (!log->vib_csv) return;
    uint64_t now = time_ns();
    fprintf(log->vib_csv, "%.3f,%d,%d,%.9f\n",
            (double)(now - t0) / 1000.0, sample_num, raw, voltage);
    log->vib_entries++;
}

static void logger_close(logger_t *log)
{
    if (log->blade_csv) fclose(log->blade_csv);
    if (log->vib_csv) fclose(log->vib_csv);
    printf("  Logs saved: turbine_blade_log.csv (%d entries), turbine_vib_log.csv (%d entries)\n",
           log->blade_entries, log->vib_entries);
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
 * Dashboard Display
 * ============================================================ */
static void display_dashboard(double elapsed_s, const blade_state_t *b,
                               const vib_state_t *v, const alert_state_t *a,
                               const adc_backend_t *adc)
{
    printf("\r  T=%6.1fs | RPM: %7.0f (avg %7.0f) | BPF: %7.0f Hz | "
           "Vib RMS: %.4f V | Vpp: %.4f V | Alerts: %d  ",
           elapsed_s, b->rpm_instant, b->rpm_avg, b->blade_freq,
           v->rms_ac, v->vpp, a->alert_count);

    /* Alert indicator */
    if (a->overspeed)       printf("[OVERSPD]");
    if (a->stall)           printf("[STALL]");
    if (a->high_vibration)  printf("[VIB!]");
    if (a->high_jitter)     printf("[JITTER]");

    fflush(stdout);
    (void)adc;
}

/* ============================================================
 * Main Monitor Loop
 *
 * Interleaves blade detection (pure GPIO polling) with vibration
 * ADC sampling. The ADC read takes ~10-30 µs in bit-bang mode,
 * during which blade edges might be missed. For production:
 * use hardware SPI + DMA for ADC, keeping the CPU free for
 * blade polling.
 *
 * Current approach: read ADC only when DRDY is asserted (non-blocking
 * check), and poll blade GPIO between ADC reads.
 * ============================================================ */
static int run_monitor(int duration_sec, int enable_logging)
{
    printf("--- Turbine Monitor ---\n");
    printf("  ADC Backend: %s (%d SPS, %d-bit)\n",
           g_active_vib_adc->name, g_active_vib_adc->max_sps,
           g_active_vib_adc->resolution);
    printf("  Blade sensor: GPIO%d (%d blades)\n", PIN_BLADE_SENSOR, NUM_BLADES);
    printf("  Duration: %d seconds (Ctrl+C to stop)\n", duration_sec);
    printf("  Alerts: overspeed >%.0f RPM, vibration >%.3f V RMS\n",
           ALERT_RPM_HIGH, ALERT_VIB_RMS_V);
    printf("\n");

    /* Initialize blade detection */
    pin_t p = pinopen(PIN_BLADE_SENSOR, INPUT);
    if (p.mode == UNDEF)
    {
        fprintf(stderr, "  FAIL: Could not open blade sensor GPIO%d\n", PIN_BLADE_SENSOR);
        return -1;
    }
    pinpull(PIN_BLADE_SENSOR, PULL_UP);

    /* Initialize ADC */
    int adc_active = 0;
    if (g_active_vib_adc->init(g_active_vib_adc) == 0)
    {
        adc_active = 1;
        g_active_vib_adc->set_channel(g_active_vib_adc, 0);
        g_active_vib_adc->start(g_active_vib_adc);
        printf("  ADC initialized successfully.\n");
    }
    else
    {
        printf("  WARNING: ADC init failed. Running blade-only mode.\n");
    }

    /* Initialize state */
    blade_state_t blade;
    vib_state_t   vib;
    alert_state_t alert;
    logger_t      logger;

    blade_init(&blade);
    vib_init(&vib);
    alert_init(&alert);

    int logging = 0;
    if (enable_logging)
    {
        if (logger_init(&logger) == 0)
            logging = 1;
        else
            printf("  WARNING: Could not initialize logger\n");
    }

    signal(SIGINT, signal_handler);

    uint64_t t_start = time_ns();
    uint64_t t_end = t_start + (uint64_t)duration_sec * 1000000000ULL;
    uint64_t last_display = t_start;
    int vib_sample_count = 0;

    printf("\n  Monitoring... (Ctrl+C to stop)\n\n");

    while (g_running && time_ns() < t_end)
    {
        uint64_t now = time_ns();

        /* === Blade detection (always poll) === */
        int blade_val = pinread(PIN_BLADE_SENSOR);
        if (blade_process(&blade, blade_val, now))
        {
            if (logging)
                logger_blade_event(&logger, t_start, &blade);
        }

        /* === Vibration ADC (non-blocking DRDY check) === */
        if (adc_active && pinread(PIN_ADC_DRDY) == LOW)
        {
            int32_t raw = g_active_vib_adc->read_raw(g_active_vib_adc);
            double voltage = g_active_vib_adc->to_voltage(g_active_vib_adc, raw);
            vib_add_sample(&vib, voltage);
            vib_sample_count++;

            if (logging && (vib_sample_count % 10) == 0) /* Log every 10th sample */
                logger_vib_sample(&logger, t_start, vib_sample_count, raw, voltage);
        }

        /* === Dashboard update === */
        if ((now - last_display) >= (uint64_t)MONITOR_INTERVAL_MS * 1000000ULL)
        {
            double elapsed = (double)(now - t_start) / 1.0e9;
            vib.actual_sps = (elapsed > 0) ? vib_sample_count / elapsed : 0;

            display_dashboard(elapsed, &blade, &vib, &alert, g_active_vib_adc);
            alert_check(&alert, &blade, &vib);
            last_display = now;
        }
    }

    /* Cleanup */
    printf("\n\n");
    if (adc_active)
    {
        g_active_vib_adc->stop(g_active_vib_adc);
        g_active_vib_adc->cleanup(g_active_vib_adc);
    }
    pinclose(PIN_BLADE_SENSOR);

    /* Final report */
    double total_s = (double)(time_ns() - t_start) / 1.0e9;

    printf("  ╔══════════════════════════════════════════════════╗\n");
    printf("  ║         Turbine Monitor — Final Report          ║\n");
    printf("  ╠══════════════════════════════════════════════════╣\n");
    printf("  ║  Duration:       %8.2f seconds                ║\n", total_s);
    printf("  ║                                                  ║\n");
    printf("  ║  BLADE DETECTION:                                ║\n");
    printf("  ║    Total pulses: %8u                         ║\n", blade.blade_count);
    printf("  ║    Revolutions:  %8u                         ║\n", blade.rev_count);
    printf("  ║    Last RPM:     %8.0f                         ║\n", blade.rpm_avg);
    printf("  ║    Blade freq:   %8.0f Hz                     ║\n", blade.blade_freq);
    printf("  ║    Max jitter:   %8.2f %%                      ║\n", blade.max_jitter_pct);
    printf("  ║                                                  ║\n");

    if (adc_active)
    {
        printf("  ║  VIBRATION (%s):                   ║\n",
               g_active_vib_adc->name);
        printf("  ║    Samples:      %8d                         ║\n", vib_sample_count);
        printf("  ║    Actual rate:  %8.0f SPS                    ║\n", vib.actual_sps);
        printf("  ║    AC RMS:       %8.6f V                   ║\n", vib.rms_ac);
        printf("  ║    Vpp:          %8.6f V                   ║\n", vib.vpp);
        printf("  ║    Mean:         %+8.6f V                   ║\n", vib.mean_v);
    }
    else
    {
        printf("  ║  VIBRATION: ADC not active                      ║\n");
    }

    printf("  ║                                                  ║\n");
    printf("  ║  ALERTS: %d total                                ║\n", alert.alert_count);
    if (alert.alert_count > 0)
        printf("  ║    Last: %-40s║\n", alert.last_msg);
    printf("  ╚══════════════════════════════════════════════════╝\n\n");

    if (logging) logger_close(&logger);

    return 0;
}

/* ============================================================
 * Simulation Mode
 *
 * Tests the framework logic without actual hardware.
 * Generates synthetic blade pulses and vibration data.
 * ============================================================ */
static int run_simulation(void)
{
    printf("--- Simulation Mode (no hardware needed) ---\n\n");

    blade_state_t blade;
    vib_state_t   vib;
    alert_state_t alert;

    blade_init(&blade);
    vib_init(&vib);
    alert_init(&alert);

    signal(SIGINT, signal_handler);

    /* Simulate 5 seconds at 100,000 RPM */
    double sim_rpm = 100000.0;
    double blade_period_ns = 60.0e9 / (sim_rpm * NUM_BLADES);
    double sim_vib_freq = 500.0; /* 500 Hz vibration */
    double sim_vib_amp = 0.1;    /* 100 mV amplitude */
    double sim_dc = 1.25;        /* DC offset */

    printf("  Simulated RPM: %.0f\n", sim_rpm);
    printf("  Blade period: %.1f µs\n", blade_period_ns / 1000.0);
    printf("  Vibration: %.0f Hz, %.0f mV amplitude\n\n", sim_vib_freq, sim_vib_amp * 1000);

    uint64_t t_start = time_ns();
    uint64_t last_display = t_start;
    uint64_t next_blade = t_start + (uint64_t)blade_period_ns;
    int vib_count = 0;

    for (int i = 0; i < 5000000 && g_running; i++) /* 5M iterations */
    {
        uint64_t now = time_ns();

        /* Simulate blade pulse */
        int gpio_val = (now >= next_blade) ? LOW : HIGH;
        if (gpio_val == LOW && blade.prev_val == HIGH)
            next_blade = now + (uint64_t)blade_period_ns;

        blade_process(&blade, gpio_val, now);

        /* Simulate vibration sample (every ~26 µs at 38.4 kSPS) */
        if ((i % 100) == 0)
        {
            double t = (double)(now - t_start) / 1.0e9;
            double v = sim_dc + sim_vib_amp * sin(2.0 * M_PI * sim_vib_freq * t);
            /* Add some noise */
            v += ((double)(rand() % 1000) - 500.0) / 1000000.0; /* ±0.5 mV */
            vib_add_sample(&vib, v);
            vib_count++;
        }

        /* Dashboard update */
        if ((now - last_display) >= 200000000ULL) /* Every 200 ms */
        {
            double elapsed = (double)(now - t_start) / 1.0e9;
            if (elapsed > 5.0) break;

            vib.actual_sps = vib_count / elapsed;
            display_dashboard(elapsed, &blade, &vib, &alert, &g_ads1263);
            alert_check(&alert, &blade, &vib);
            last_display = now;
        }
    }

    printf("\n\n  Simulation complete.\n");
    printf("    Blade pulses: %u, Revolutions: %u\n", blade.blade_count, blade.rev_count);
    printf("    Average RPM:  %.0f (target: %.0f)\n", blade.rpm_avg, sim_rpm);
    printf("    Vib AC RMS:   %.4f V (expected: ~%.4f V)\n",
           vib.rms_ac, sim_vib_amp / sqrt(2.0));
    printf("    Alerts: %d\n\n", alert.alert_count);

    double rpm_error = fabs(blade.rpm_avg - sim_rpm) / sim_rpm * 100.0;
    if (rpm_error < 5.0)
        printf("  PASS: RPM measurement within %.1f%% of target\n", rpm_error);
    else
        printf("  FAIL: RPM error %.1f%% — check timing logic\n", rpm_error);

    printf("\n");
    return 0;
}

/* ============================================================ */

static void print_usage(const char *prog)
{
    printf("Usage: sudo %s [options]\n", prog);
    printf("Options:\n");
    printf("  (none)    Monitor for 30 seconds\n");
    printf("  <N>       Monitor for N seconds\n");
    printf("  log       Monitor + log to CSV\n");
    printf("  sim       Simulation mode (no hardware)\n");
    printf("  sar       Try SAR ADC backend (placeholder)\n");
    printf("  help      Show this message\n");
}

int main(int argc, char *argv[])
{
    int duration = 30;
    int do_sim = 0;
    int do_log = 0;
    int use_sar = 0;

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "sim") == 0)         do_sim = 1;
        else if (strcmp(argv[i], "log") == 0)    do_log = 1;
        else if (strcmp(argv[i], "sar") == 0)    use_sar = 1;
        else if (strcmp(argv[i], "help") == 0 || strcmp(argv[i], "-h") == 0)
        {
            print_usage(argv[0]);
            return 0;
        }
        else if (argv[i][0] >= '0' && argv[i][0] <= '9')
        {
            duration = atoi(argv[i]);
            if (duration < 1) duration = 1;
            if (duration > 3600) duration = 3600;
        }
        else
        {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    printf("═══════════════════════════════════════════════════\n");
    printf("  Turbine Monitoring Framework\n");
    printf("  Blade Detection + Vibration + Alert System\n");
    printf("  GPIO_RPI5 Library\n");
    printf("═══════════════════════════════════════════════════\n\n");

    /* Simulation mode — no GPIO needed */
    if (do_sim)
        return run_simulation();

    /* Select ADC backend */
    if (use_sar)
        g_active_vib_adc = &g_sar_adc;

    /* Initialize GPIO */
    if (gpio_init() < 0)
    {
        fprintf(stderr, "FATAL: gpio_init() failed. Are you root on a Raspberry Pi 5?\n");
        return 1;
    }

    int result = run_monitor(duration, do_log);

    gpio_cleanup();
    return result;
}
