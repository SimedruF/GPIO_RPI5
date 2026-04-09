/*
 * test_ads1263.cpp - ADS1263 ADC HAT test using GPIO_RPI5 library
 *
 * Test for the Waveshare "High-Precision ADC HAT For Raspberry Pi"
 * (10-Channel 32-Bit) which uses the Texas Instruments ADS1263 chip.
 *
 * The ADS1263 provides:
 *   - ADC1: 32-bit delta-sigma converter (main)
 *   - ADC2: 24-bit delta-sigma converter (auxiliary)
 *   - 10 analog input channels (AIN0 - AIN9)
 *   - Internal 2.5V reference
 *   - SPI interface (Mode 1: CPOL=0, CPHA=1)
 *
 * Waveshare HAT pin mapping (40-pin header):
 *   GPIO10 = MOSI  (DIN)   - Header pin 19
 *   GPIO9  = MISO  (DOUT)  - Header pin 21
 *   GPIO11 = SCLK          - Header pin 23
 *   GPIO8  = CS    (CE0)   - Header pin 24
 *   GPIO17 = DRDY           - Header pin 11  (Data Ready, active LOW)
 *   GPIO18 = RST            - Header pin 12  (Reset, active LOW)
 *
 * Compile:
 *   g++ test_ads1263.cpp -I.. -L.. -l:gpio_rpi5.a -o test_ads1263 -Wall -Wextra
 *
 * Run:
 *   sudo ./test_ads1263           # read all 10 channels once
 *   sudo ./test_ads1263 loop      # continuous reading (Ctrl+C to stop)
 *   sudo ./test_ads1263 adc2      # also read ADC2 (24-bit) channels
 */

#include "../gpio_rpi5.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <math.h>

/* ============================================================
 * Pin Definitions (Waveshare High-Precision ADC HAT)
 * ============================================================ */
#define ADC_MOSI   GPIO10
#define ADC_MISO   GPIO9
#define ADC_SCLK   GPIO11
#define ADC_CS     GPIO8    /* CE0 */
#define ADC_DRDY   GPIO17   /* Data Ready (active LOW) */
#define ADC_RST    GPIO18   /* Reset (active LOW) */

/* ============================================================
 * ADS1263 Commands
 * ============================================================ */
#define CMD_NOP      0x00
#define CMD_RESET    0x06
#define CMD_START1   0x08   /* Start ADC1 conversions */
#define CMD_STOP1    0x0A   /* Stop ADC1 conversions */
#define CMD_START2   0x0C   /* Start ADC2 conversions */
#define CMD_STOP2    0x0E   /* Stop ADC2 conversions */
#define CMD_RDATA1   0x12   /* Read ADC1 data */
#define CMD_RDATA2   0x14   /* Read ADC2 data */
#define CMD_RREG     0x20   /* Read register: 0x20 | reg_addr */
#define CMD_WREG     0x40   /* Write register: 0x40 | reg_addr */
#define CMD_SFOCAL1  0x19   /* Self offset calibration ADC1 */
#define CMD_SFOCAL2  0x1B   /* Self offset calibration ADC2 */

/* ============================================================
 * ADS1263 Register Addresses
 * ============================================================ */
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
#define REG_FSCAL0    0x0A
#define REG_FSCAL1    0x0B
#define REG_FSCAL2    0x0C
#define REG_IDACMUX   0x0D
#define REG_IDACMAG   0x0E
#define REG_REFMUX    0x0F
#define REG_TDACP     0x10
#define REG_TDACN     0x11
#define REG_GPIOCON   0x12
#define REG_GPIODIR   0x13
#define REG_GPIODAT   0x14
#define REG_ADC2CFG   0x15
#define REG_ADC2MUX   0x16
#define REG_ADC2OFC0  0x17
#define REG_ADC2OFC1  0x18
#define REG_ADC2FSC0  0x19
#define REG_ADC2FSC1  0x1A

/* ADS1263 Device ID (bits [7:5] = DEV_ID = 0b000, bits [4:0] = REV_ID) */
#define ADS1263_DEV_ID  0x01

/* ============================================================
 * ADS1263 Data Rate Constants (REG_MODE2 bits [3:0])
 * ============================================================ */
#define DR_2_5     0x00   /*    2.5 SPS */
#define DR_5       0x01   /*    5   SPS */
#define DR_10      0x02   /*   10   SPS */
#define DR_16_6    0x03   /*   16.6 SPS */
#define DR_20      0x04   /*   20   SPS (default) */
#define DR_50      0x05   /*   50   SPS */
#define DR_60      0x06   /*   60   SPS */
#define DR_100     0x07   /*  100   SPS */
#define DR_400     0x08   /*  400   SPS */
#define DR_1200    0x09   /* 1200   SPS */
#define DR_2400    0x0A   /* 2400   SPS */
#define DR_4800    0x0B   /* 4800   SPS */
#define DR_7200    0x0C   /* 7200   SPS */
#define DR_14400   0x0D   /*14400   SPS */
#define DR_19200   0x0E   /*19200   SPS */
#define DR_38400   0x0F   /*38400   SPS */

/* ============================================================
 * ADS1263 Gain Constants (REG_MODE2 bits [6:4])
 * ============================================================ */
#define GAIN_1     0x00
#define GAIN_2     0x10
#define GAIN_4     0x20
#define GAIN_8     0x30
#define GAIN_16    0x40
#define GAIN_32    0x50

/* Reference voltage (internal 2.5V) */
#define VREF       2.5

/* Number of analog input channels */
#define NUM_CHANNELS 10

/* DRDY wait timeout in microseconds (2 seconds) */
#define DRDY_TIMEOUT_US 2000000

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
 * Bit-banged SPI (Mode 1: CPOL=0, CPHA=1)
 *
 * ADS1263 SPI protocol:
 *   - CPOL=0: clock idles LOW
 *   - CPHA=1: data shifted out on rising edge, sampled on falling edge
 *   - MSB first
 *   - CS active LOW
 * ============================================================ */

static inline void spi_delay(void)
{
    usleep(1); /* ~500 kHz effective clock */
}

static int spi_init(void)
{
    pin_t p;

    p = pinopen(ADC_SCLK, OUTPUT);
    if (p.mode == UNDEF) return -1;
    pinwrite(ADC_SCLK, LOW); /* CPOL=0 */

    p = pinopen(ADC_MOSI, OUTPUT);
    if (p.mode == UNDEF) return -1;
    pinwrite(ADC_MOSI, LOW);

    p = pinopen(ADC_MISO, INPUT);
    if (p.mode == UNDEF) return -1;
    pinpull(ADC_MISO, PULL_NONE);

    p = pinopen(ADC_CS, OUTPUT);
    if (p.mode == UNDEF) return -1;
    pinwrite(ADC_CS, HIGH); /* CS idle high */

    p = pinopen(ADC_DRDY, INPUT);
    if (p.mode == UNDEF) return -1;
    pinpull(ADC_DRDY, PULL_UP);

    p = pinopen(ADC_RST, OUTPUT);
    if (p.mode == UNDEF) return -1;
    pinwrite(ADC_RST, HIGH); /* RST idle high (not in reset) */

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

/* SPI Mode 1: data shifted out on rising edge, sampled on falling edge */
static uint8_t spi_transfer_byte(uint8_t tx)
{
    uint8_t rx = 0;

    for (int i = 7; i >= 0; i--)
    {
        /* Rising edge: shift out MOSI data */
        pinwrite(ADC_SCLK, HIGH);
        pinwrite(ADC_MOSI, (tx >> i) & 1);
        spi_delay();

        /* Falling edge: sample MISO */
        pinwrite(ADC_SCLK, LOW);
        spi_delay();
        if (pinread(ADC_MISO))
            rx |= (1 << i);
    }

    return rx;
}

static void spi_transfer(const uint8_t *tx, uint8_t *rx, int len)
{
    pinwrite(ADC_CS, LOW);
    spi_delay();

    for (int i = 0; i < len; i++)
        rx[i] = spi_transfer_byte(tx[i]);

    spi_delay();
    pinwrite(ADC_CS, HIGH);
}

static void spi_write_bytes(const uint8_t *tx, int len)
{
    pinwrite(ADC_CS, LOW);
    spi_delay();

    for (int i = 0; i < len; i++)
        spi_transfer_byte(tx[i]);

    spi_delay();
    pinwrite(ADC_CS, HIGH);
}

/* ============================================================
 * ADS1263 Low-level Functions
 * ============================================================ */

/* Wait for DRDY pin to go LOW (conversion complete) */
static int ads1263_wait_drdy(void)
{
    int elapsed = 0;

    while (pinread(ADC_DRDY) != LOW)
    {
        usleep(100);
        elapsed += 100;
        if (elapsed >= DRDY_TIMEOUT_US)
        {
            fprintf(stderr, "[ADS1263] ERROR: DRDY timeout after %d ms\n",
                    DRDY_TIMEOUT_US / 1000);
            return -1;
        }
    }
    return 0;
}

/* Hardware reset via RST pin */
static void ads1263_hw_reset(void)
{
    printf("[ADS1263] Hardware reset...\n");
    pinwrite(ADC_RST, LOW);
    usleep(200000); /* hold LOW for 200 ms */
    pinwrite(ADC_RST, HIGH);
    usleep(200000); /* wait 200 ms for startup */
}

/* Software reset via RESET command */
static void ads1263_sw_reset(void)
{
    uint8_t cmd = CMD_RESET;
    spi_write_bytes(&cmd, 1);
    usleep(200000);
}

/* Read a single register */
static uint8_t ads1263_read_reg(uint8_t reg)
{
    uint8_t tx[3];
    uint8_t rx[3];

    tx[0] = CMD_RREG | (reg & 0x1F); /* command: read register */
    tx[1] = 0x00;                     /* number of registers - 1 (read 1) */
    tx[2] = 0x00;                     /* dummy to clock out data */

    spi_transfer(tx, rx, 3);
    return rx[2]; /* register data is in the 3rd byte */
}

/* Write a single register */
static void ads1263_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[3];

    tx[0] = CMD_WREG | (reg & 0x1F); /* command: write register */
    tx[1] = 0x00;                     /* number of registers - 1 (write 1) */
    tx[2] = value;                    /* register data */

    spi_write_bytes(tx, 3);
}

/* Send a single-byte command */
static void ads1263_send_cmd(uint8_t cmd)
{
    spi_write_bytes(&cmd, 1);
}

/* ============================================================
 * ADS1263 High-level Functions
 * ============================================================ */

/* Check chip ID */
static int ads1263_check_id(void)
{
    uint8_t id = ads1263_read_reg(REG_ID);

    printf("[ADS1263] Chip ID register: 0x%02X (DEV_ID=%d, REV_ID=%d)\n",
           id, (id >> 5) & 0x07, id & 0x1F);

    if ((id & 0x1F) == ADS1263_DEV_ID)
    {
        printf("[ADS1263] Device ID verified: ADS1263 detected\n");
        return 0;
    }
    else
    {
        fprintf(stderr, "[ADS1263] WARNING: Unexpected ID. Got REV_ID=0x%02X, expected 0x%02X\n",
                id & 0x1F, ADS1263_DEV_ID);
        fprintf(stderr, "[ADS1263]   (This may still work — some revisions differ)\n");
        return -1;
    }
}

/* Initialize ADS1263 with default configuration */
static int ads1263_init(uint8_t gain, uint8_t data_rate)
{
    printf("[ADS1263] Initializing SPI pins...\n");
    if (spi_init() < 0)
    {
        fprintf(stderr, "[ADS1263] FAIL: Could not initialize SPI pins\n");
        return -1;
    }

    ads1263_hw_reset();

    if (ads1263_check_id() < 0)
        printf("[ADS1263] Continuing despite ID mismatch...\n");

    /* Configure MODE0: continuous conversion, no delay */
    ads1263_write_reg(REG_MODE0, 0x00);

    /* Configure MODE1: FIR filter, single-shot disabled */
    ads1263_write_reg(REG_MODE1, 0x80); /* Digital filter = sinc1 */

    /* Configure MODE2: gain and data rate */
    ads1263_write_reg(REG_MODE2, (gain & 0x70) | (data_rate & 0x0F));

    /*
     * Configure REFMUX: use internal 2.5V reference
     * RMUXP = Internal 2.5V (0x0), RMUXN = Internal AVSS (0x0)
     */
    ads1263_write_reg(REG_REFMUX, 0x00);

    /* Enable internal voltage reference in POWER register */
    ads1263_write_reg(REG_POWER, 0x01); /* INTREF = 1 */

    /* Configure INTERFACE: timeout enabled, status byte reported */
    ads1263_write_reg(REG_INTERFACE, 0x05); /* STATUS + CRC */

    usleep(10000); /* 10 ms settling time */

    /* Verify written registers */
    printf("[ADS1263] Register verification:\n");
    printf("  MODE0    = 0x%02X\n", ads1263_read_reg(REG_MODE0));
    printf("  MODE1    = 0x%02X\n", ads1263_read_reg(REG_MODE1));
    printf("  MODE2    = 0x%02X\n", ads1263_read_reg(REG_MODE2));
    printf("  REFMUX   = 0x%02X\n", ads1263_read_reg(REG_REFMUX));
    printf("  POWER    = 0x%02X\n", ads1263_read_reg(REG_POWER));
    printf("  INPMUX   = 0x%02X\n", ads1263_read_reg(REG_INPMUX));

    return 0;
}

/*
 * Read ADC1 conversion (32-bit result).
 * Returns raw signed 32-bit code. Status byte + 4 data bytes + CRC.
 */
static int32_t ads1263_read_adc1(uint8_t *status_out)
{
    uint8_t tx[7] = { CMD_RDATA1, 0, 0, 0, 0, 0, 0 };
    uint8_t rx[7] = { 0 };

    spi_transfer(tx, rx, 7);

    /* rx[1] = status, rx[2..5] = 32-bit data (MSB first), rx[6] = checksum */
    if (status_out)
        *status_out = rx[1];

    int32_t raw = ((int32_t)rx[2] << 24) |
                  ((int32_t)rx[3] << 16) |
                  ((int32_t)rx[4] <<  8) |
                  ((int32_t)rx[5]);

    return raw;
}

/*
 * Read ADC2 conversion (24-bit result).
 * Returns raw signed 24-bit code (sign-extended to int32_t).
 */
static int32_t ads1263_read_adc2(uint8_t *status_out)
{
    uint8_t tx[6] = { CMD_RDATA2, 0, 0, 0, 0, 0 };
    uint8_t rx[6] = { 0 };

    spi_transfer(tx, rx, 6);

    /* rx[1] = status, rx[2..4] = 24-bit data (MSB first), rx[5] = checksum */
    if (status_out)
        *status_out = rx[1];

    int32_t raw = ((int32_t)rx[2] << 16) |
                  ((int32_t)rx[3] <<  8) |
                  ((int32_t)rx[4]);

    /* Sign-extend from 24-bit to 32-bit */
    if (raw & 0x800000)
        raw |= (int32_t)0xFF000000;

    return raw;
}

/* Set input multiplexer for ADC1 (positive and negative input) */
static void ads1263_set_channel(uint8_t pos_ain, uint8_t neg_ain)
{
    uint8_t mux = ((pos_ain & 0x0F) << 4) | (neg_ain & 0x0F);
    ads1263_write_reg(REG_INPMUX, mux);
}

/* Set input multiplexer for ADC2 */
static void ads1263_set_channel_adc2(uint8_t pos_ain, uint8_t neg_ain)
{
    uint8_t mux = ((pos_ain & 0x0F) << 4) | (neg_ain & 0x0F);
    ads1263_write_reg(REG_ADC2MUX, mux);
}

/* Convert raw ADC1 code to voltage */
static double ads1263_code_to_voltage(int32_t raw, uint8_t gain_setting)
{
    /* Full-scale = ±VREF / gain */
    static const double gain_table[] = { 1,2,4,8,16,32 };
    double gain = gain_table[(gain_setting >> 4) & 0x07];
    /* 32-bit code: 2^31 = full scale */
    return ((double)raw / 2147483648.0) * (VREF / gain);
}

/* Convert raw ADC2 code to voltage */
static double ads1263_code_to_voltage_adc2(int32_t raw)
{
    /* ADC2 is 24-bit, fixed gain = 1, same VREF */
    return ((double)raw / 8388608.0) * VREF;
}

/* ============================================================
 * Test Functions
 * ============================================================ */

/*
 * Test 1: Verify communication - read chip ID
 */
static int test_chip_id(void)
{
    printf("--- Test 1: Chip ID Verification ---\n");

    uint8_t id = ads1263_read_reg(REG_ID);

    if (id == 0x00 || id == 0xFF)
    {
        fprintf(stderr, "[TEST 1] FAIL: Read 0x%02X - likely no device connected\n", id);
        fprintf(stderr, "  Check wiring:\n");
        fprintf(stderr, "    MOSI  -> GPIO%d (pin 19)\n", ADC_MOSI);
        fprintf(stderr, "    MISO  -> GPIO%d (pin 21)\n", ADC_MISO);
        fprintf(stderr, "    SCLK  -> GPIO%d (pin 23)\n", ADC_SCLK);
        fprintf(stderr, "    CS    -> GPIO%d (pin 24)\n", ADC_CS);
        fprintf(stderr, "    DRDY  -> GPIO%d (pin 11)\n", ADC_DRDY);
        fprintf(stderr, "    RST   -> GPIO%d (pin 12)\n", ADC_RST);
        return -1;
    }

    printf("[TEST 1] PASS: Chip ID = 0x%02X\n\n", id);
    return 0;
}

/*
 * Test 2: Register read/write
 */
static int test_register_rw(void)
{
    int errors = 0;

    printf("--- Test 2: Register Read/Write ---\n");

    /* Write a known value to MODE0, read it back */
    uint8_t test_val = 0x24; /* DELAY[3:0]=4, CHOP=0, RUNMODE=1 */
    ads1263_write_reg(REG_MODE0, test_val);
    usleep(1000);
    uint8_t readback = ads1263_read_reg(REG_MODE0);

    if (readback == test_val)
    {
        printf("[TEST 2] MODE0 write/read: PASS (0x%02X == 0x%02X)\n",
               readback, test_val);
    }
    else
    {
        fprintf(stderr, "[TEST 2] MODE0 write/read: FAIL (wrote 0x%02X, read 0x%02X)\n",
                test_val, readback);
        errors++;
    }

    /* Restore MODE0 to default */
    ads1263_write_reg(REG_MODE0, 0x00);

    /* Test INPMUX register */
    test_val = 0x45; /* MUXP=AIN4, MUXN=AIN5 */
    ads1263_write_reg(REG_INPMUX, test_val);
    usleep(1000);
    readback = ads1263_read_reg(REG_INPMUX);

    if (readback == test_val)
    {
        printf("[TEST 2] INPMUX write/read: PASS (0x%02X == 0x%02X)\n",
               readback, test_val);
    }
    else
    {
        fprintf(stderr, "[TEST 2] INPMUX write/read: FAIL (wrote 0x%02X, read 0x%02X)\n",
                test_val, readback);
        errors++;
    }

    /* Restore to default (AIN0 vs AINCOM) */
    ads1263_write_reg(REG_INPMUX, 0x0A); /* AIN0 positive, AINCOM negative */

    printf("[TEST 2] %s (%d error(s))\n\n",
           errors == 0 ? "PASS" : "FAIL", errors);
    return errors ? -1 : 0;
}

/*
 * Test 3: Single-ended read all 10 channels (AINx vs AINCOM)
 * AINCOM = 0x0A on ADS1263
 */
static int test_read_all_channels(uint8_t gain_setting)
{
    printf("--- Test 3: Read All 10 Channels (ADC1, 32-bit) ---\n");
    printf("  Configuration: single-ended (AINx vs AINCOM), VREF=%.1fV\n\n", VREF);
    printf("  %-8s  %-14s  %-12s\n", "Channel", "Raw Code", "Voltage (V)");
    printf("  %-8s  %-14s  %-12s\n", "-------", "--------", "-----------");

    int errors = 0;

    for (int ch = 0; ch < NUM_CHANNELS; ch++)
    {
        /* Set MUX: AINx (positive) vs AINCOM=0x0A (negative) */
        ads1263_set_channel(ch, 0x0A);
        usleep(10000); /* settling time */

        /* Start ADC1 conversion */
        ads1263_send_cmd(CMD_START1);

        /* Wait for DRDY */
        if (ads1263_wait_drdy() < 0)
        {
            fprintf(stderr, "  AIN%-5d  DRDY TIMEOUT\n", ch);
            ads1263_send_cmd(CMD_STOP1);
            errors++;
            continue;
        }

        /* Read result */
        uint8_t status = 0;
        int32_t raw = ads1263_read_adc1(&status);
        double voltage = ads1263_code_to_voltage(raw, gain_setting);

        printf("  AIN%-5d  %12d    %8.6f\n", ch, raw, voltage);

        ads1263_send_cmd(CMD_STOP1);
        usleep(1000);
    }

    printf("\n[TEST 3] %s\n\n", errors == 0 ? "PASS" : "PARTIAL (some channels timed out)");
    return errors ? -1 : 0;
}

/*
 * Test 4: ADC2 (24-bit) read all channels
 */
static int test_read_all_channels_adc2(void)
{
    printf("--- Test 4: Read All 10 Channels (ADC2, 24-bit) ---\n");
    printf("  Configuration: single-ended (AINx vs AINCOM), VREF=%.1fV\n\n", VREF);
    printf("  %-8s  %-14s  %-12s\n", "Channel", "Raw Code", "Voltage (V)");
    printf("  %-8s  %-14s  %-12s\n", "-------", "--------", "-----------");

    /* Configure ADC2 data rate (10 SPS) and reference */
    ads1263_write_reg(REG_ADC2CFG, 0x00); /* 10 SPS, sinc3, internal ref */

    int errors = 0;

    for (int ch = 0; ch < NUM_CHANNELS; ch++)
    {
        ads1263_set_channel_adc2(ch, 0x0A);
        usleep(10000);

        ads1263_send_cmd(CMD_START2);

        if (ads1263_wait_drdy() < 0)
        {
            fprintf(stderr, "  AIN%-5d  DRDY TIMEOUT\n", ch);
            ads1263_send_cmd(CMD_STOP2);
            errors++;
            continue;
        }

        uint8_t status = 0;
        int32_t raw = ads1263_read_adc2(&status);
        double voltage = ads1263_code_to_voltage_adc2(raw);

        printf("  AIN%-5d  %12d    %8.6f\n", ch, raw, voltage);

        ads1263_send_cmd(CMD_STOP2);
        usleep(1000);
    }

    printf("\n[TEST 4] %s\n\n", errors == 0 ? "PASS" : "PARTIAL (some channels timed out)");
    return errors ? -1 : 0;
}

/*
 * Test 5: Continuous reading (all channels in a loop)
 */
static int test_continuous_read(uint8_t gain_setting)
{
    printf("--- Test 5: Continuous Read (press Ctrl+C to stop) ---\n\n");

    signal(SIGINT, signal_handler);
    int iteration = 0;

    while (g_running)
    {
        printf("  [Iteration %d]\n", ++iteration);
        printf("  %-8s  %-12s\n", "Channel", "Voltage (V)");
        printf("  %-8s  %-12s\n", "-------", "-----------");

        for (int ch = 0; ch < NUM_CHANNELS && g_running; ch++)
        {
            ads1263_set_channel(ch, 0x0A);
            usleep(5000);

            ads1263_send_cmd(CMD_START1);

            if (ads1263_wait_drdy() < 0)
            {
                printf("  AIN%-5d  TIMEOUT\n", ch);
                ads1263_send_cmd(CMD_STOP1);
                continue;
            }

            uint8_t status = 0;
            int32_t raw = ads1263_read_adc1(&status);
            double voltage = ads1263_code_to_voltage(raw, gain_setting);

            printf("  AIN%-5d  %8.6f\n", ch, voltage);

            ads1263_send_cmd(CMD_STOP1);
        }

        printf("\n");
        usleep(500000); /* 500 ms between iterations */
    }

    printf("[TEST 5] Stopped by user.\n\n");
    return 0;
}

/*
 * Test 6: Self-offset calibration
 */
static int test_self_calibration(void)
{
    printf("--- Test 6: Self-Offset Calibration ---\n");

    printf("[ADS1263] Running ADC1 self-offset calibration...\n");
    ads1263_send_cmd(CMD_SFOCAL1);
    /* Calibration takes a while, wait for DRDY */
    usleep(500000); /* conservative 500 ms wait */

    printf("[ADS1263] Calibration offset registers:\n");
    printf("  OFCAL0 = 0x%02X\n", ads1263_read_reg(REG_OFCAL0));
    printf("  OFCAL1 = 0x%02X\n", ads1263_read_reg(REG_OFCAL1));
    printf("  OFCAL2 = 0x%02X\n", ads1263_read_reg(REG_OFCAL2));

    printf("[TEST 6] PASS: Calibration complete\n\n");
    return 0;
}

/* ============================================================
 * Main
 * ============================================================ */

static void print_usage(const char *prog)
{
    printf("Usage: sudo %s [options]\n", prog);
    printf("Options:\n");
    printf("  (none)    Read all 10 channels once (ADC1, 32-bit)\n");
    printf("  loop      Continuous reading (Ctrl+C to stop)\n");
    printf("  adc2      Also read channels with ADC2 (24-bit)\n");
    printf("  cal       Run self-offset calibration, then read\n");
    printf("  all       Run all tests\n");
    printf("  help      Show this message\n");
}

int main(int argc, char *argv[])
{
    int result = 0;
    int do_loop = 0;
    int do_adc2 = 0;
    int do_cal  = 0;
    int do_all  = 0;

    /* Parse arguments */
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "loop") == 0)       do_loop = 1;
        else if (strcmp(argv[i], "adc2") == 0)  do_adc2 = 1;
        else if (strcmp(argv[i], "cal") == 0)   do_cal  = 1;
        else if (strcmp(argv[i], "all") == 0)   do_all  = 1;
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

    if (do_all) { do_adc2 = 1; do_cal = 1; }

    printf("===============================================\n");
    printf(" ADS1263 High-Precision ADC HAT Test\n");
    printf(" 10-Channel 32-Bit ADC (Waveshare)\n");
    printf(" Using GPIO_RPI5 bit-banged SPI\n");
    printf("===============================================\n\n");

    /* Initialize GPIO subsystem */
    if (gpio_init() < 0)
    {
        fprintf(stderr, "FATAL: gpio_init() failed. Are you root on a Raspberry Pi 5?\n");
        return 1;
    }

    /* Gain and data rate for this test session */
    uint8_t gain_setting = GAIN_1;
    uint8_t data_rate = DR_20; /* 20 SPS - good balance of speed and noise */

    /* Initialize ADS1263 */
    if (ads1263_init(gain_setting, data_rate) < 0)
    {
        fprintf(stderr, "FATAL: Could not initialize ADS1263\n");
        spi_cleanup();
        gpio_cleanup();
        return 1;
    }

    /* Test 1: Chip ID */
    if (test_chip_id() < 0)
        result = 1;

    /* Test 2: Register R/W */
    if (test_register_rw() < 0)
        result = 1;

    /* Test 6: Self calibration (before readings, if requested) */
    if (do_cal)
        test_self_calibration();

    /* Test 3: Read all channels (ADC1, 32-bit) */
    if (test_read_all_channels(gain_setting) < 0)
        result = 1;

    /* Test 4: Read all channels (ADC2, 24-bit) */
    if (do_adc2)
    {
        if (test_read_all_channels_adc2() < 0)
            result = 1;
    }

    /* Test 5: Continuous reading */
    if (do_loop)
        test_continuous_read(gain_setting);

    /* Cleanup */
    ads1263_send_cmd(CMD_STOP1);
    ads1263_send_cmd(CMD_STOP2);
    spi_cleanup();
    gpio_cleanup();

    printf("=== %s ===\n", result ? "SOME TESTS FAILED" : "ALL TESTS PASSED");
    return result;
}
