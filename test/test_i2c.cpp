/*
 * test_i2c.cpp - I2C test using GPIO_RPI5 library
 *
 * Demonstrates two I2C modes:
 *   1. Hardware I2C: configure pins to ALT3 for I2C1 peripheral
 *   2. Bit-banged I2C: manual I2C master via GPIO
 *
 * RPi5 I2C1 pin mapping (40-pin header):
 *   GPIO2 = SDA1 (ALT3) - Header pin 3
 *   GPIO3 = SCL1 (ALT3) - Header pin 5
 *
 * Compile:
 *   g++ test_i2c.cpp -I.. -L.. -l:gpio_rpi5.a -o test_i2c
 *
 * Run:
 *   sudo ./test_i2c
 *   sudo ./test_i2c scan     # scan I2C bus for devices
 */

#include "../gpio_rpi5.h"
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>

/* I2C1 pin definitions */
#define I2C_SDA   GPIO2   /* Header pin 3 */
#define I2C_SCL   GPIO3   /* Header pin 5 */

/* I2C timing (microseconds) */
#define I2C_DELAY_US  5   /* ~100kHz standard mode */

/* ============================================================
 * Test 1: Hardware I2C pin configuration
 * Configures I2C1 pins to ALT3 for the RP1 I2C peripheral.
 * After this, use /dev/i2c-1 for actual I2C transfers.
 * ============================================================ */
static int test_hw_i2c_setup(void)
{
    int rc = 0;

    printf("[HW I2C] Configuring I2C1 pins to ALT3...\n");

    if (pinalt(I2C_SDA, ALT3) < 0) {
        fprintf(stderr, "[HW I2C] FAIL: Could not set GPIO%d to ALT3 (SDA1)\n", I2C_SDA);
        rc = -1;
    } else {
        printf("[HW I2C]   GPIO%d -> ALT3 (SDA1) OK  [Header pin 3]\n", I2C_SDA);
    }

    if (pinalt(I2C_SCL, ALT3) < 0) {
        fprintf(stderr, "[HW I2C] FAIL: Could not set GPIO%d to ALT3 (SCL1)\n", I2C_SCL);
        rc = -1;
    } else {
        printf("[HW I2C]   GPIO%d -> ALT3 (SCL1) OK  [Header pin 5]\n", I2C_SCL);
    }

    /* I2C needs pull-ups (usually external 4.7k, but internal pull-ups help) */
    pinpull(I2C_SDA, PULL_UP);
    pinpull(I2C_SCL, PULL_UP);

    if (rc == 0) {
        printf("[HW I2C] I2C1 pins configured.\n");
        printf("[HW I2C] Use /dev/i2c-1 for transfers:\n");
        printf("[HW I2C]   i2cdetect -y 1       # scan bus\n");
        printf("[HW I2C]   i2cget -y 1 0x48 0   # read device\n");
    }

    return rc;
}

static void test_hw_i2c_cleanup(void)
{
    printf("[HW I2C] Restoring I2C pins to GPIO mode...\n");
    pinalt(I2C_SDA, GPIO_FUNC_SIO);
    pinalt(I2C_SCL, GPIO_FUNC_SIO);
    printf("[HW I2C] Done.\n");
}

/* ============================================================
 * Test 2: Bit-banged I2C master
 * Manual I2C implementation using GPIO.
 * Open-drain emulation: output LOW to pull down, switch to
 * input (high-Z) + pull-up for HIGH.
 * ============================================================ */

static inline void i2c_delay(void)
{
    usleep(I2C_DELAY_US);
}

/* Release SDA (let pull-up bring it HIGH) */
static void sda_high(void)
{
    pinopen(I2C_SDA, INPUT);
    pinpull(I2C_SDA, PULL_UP);
}

/* Drive SDA LOW */
static void sda_low(void)
{
    pinopen(I2C_SDA, OUTPUT);
    pinwrite(I2C_SDA, LOW);
}

/* Read SDA level */
static int sda_read(void)
{
    pinopen(I2C_SDA, INPUT);
    pinpull(I2C_SDA, PULL_UP);
    return pinread(I2C_SDA);
}

/* Release SCL (let pull-up bring it HIGH) */
static void scl_high(void)
{
    pinopen(I2C_SCL, INPUT);
    pinpull(I2C_SCL, PULL_UP);
}

/* Drive SCL LOW */
static void scl_low(void)
{
    pinopen(I2C_SCL, OUTPUT);
    pinwrite(I2C_SCL, LOW);
}

/* I2C START condition: SDA goes LOW while SCL is HIGH */
static void i2c_start(void)
{
    sda_high();
    scl_high();
    i2c_delay();
    sda_low();
    i2c_delay();
    scl_low();
    i2c_delay();
}

/* I2C STOP condition: SDA goes HIGH while SCL is HIGH */
static void i2c_stop(void)
{
    sda_low();
    i2c_delay();
    scl_high();
    i2c_delay();
    sda_high();
    i2c_delay();
}

/* Write one byte, MSB first. Returns ACK bit (0=ACK, 1=NACK). */
static int i2c_write_byte(uint8_t byte)
{
    int i, ack;

    for (i = 7; i >= 0; i--)
    {
        if (byte & (1 << i))
            sda_high();
        else
            sda_low();

        i2c_delay();
        scl_high();
        i2c_delay();
        scl_low();
        i2c_delay();
    }

    /* Read ACK: release SDA, clock in ACK bit */
    sda_high();
    i2c_delay();
    scl_high();
    i2c_delay();
    ack = sda_read();  /* 0 = ACK (slave pulled low), 1 = NACK */
    scl_low();
    i2c_delay();

    return ack;
}

/* Read one byte. Send ACK (0) or NACK (1) after. */
static uint8_t i2c_read_byte(int send_nack)
{
    uint8_t byte = 0;
    int i;

    sda_high();  /* Release SDA so slave can drive it */

    for (i = 7; i >= 0; i--)
    {
        scl_high();
        i2c_delay();
        if (sda_read())
            byte |= (1 << i);
        scl_low();
        i2c_delay();
    }

    /* Send ACK or NACK */
    if (send_nack)
        sda_high();  /* NACK */
    else
        sda_low();   /* ACK */

    i2c_delay();
    scl_high();
    i2c_delay();
    scl_low();
    i2c_delay();
    sda_high();

    return byte;
}

/* Probe a 7-bit I2C address. Returns 0 if device responds with ACK. */
static int i2c_probe(uint8_t addr)
{
    int ack;

    i2c_start();
    ack = i2c_write_byte((addr << 1) | 0);  /* Write mode */
    i2c_stop();

    return (ack == 0) ? 0 : -1;  /* 0 = found, -1 = no response */
}

/* Write one byte to a register on device at addr */
static int i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value)
{
    int ack;

    i2c_start();
    ack = i2c_write_byte((addr << 1) | 0);
    if (ack) { i2c_stop(); return -1; }

    ack = i2c_write_byte(reg);
    if (ack) { i2c_stop(); return -1; }

    ack = i2c_write_byte(value);
    i2c_stop();

    return ack ? -1 : 0;
}

/* Read one byte from a register on device at addr */
static int i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *value)
{
    int ack;

    /* Write register address */
    i2c_start();
    ack = i2c_write_byte((addr << 1) | 0);
    if (ack) { i2c_stop(); return -1; }

    ack = i2c_write_byte(reg);
    if (ack) { i2c_stop(); return -1; }

    /* Repeated start, read mode */
    i2c_start();
    ack = i2c_write_byte((addr << 1) | 1);
    if (ack) { i2c_stop(); return -1; }

    *value = i2c_read_byte(1);  /* NACK after last byte */
    i2c_stop();

    return 0;
}

/* ============================================================
 * Test: Scan I2C bus (like i2cdetect)
 * ============================================================ */
static int test_i2c_scan(void)
{
    int addr, found = 0;

    printf("[BB I2C] Scanning I2C bus (bit-banged on GPIO%d/GPIO%d)...\n",
           I2C_SDA, I2C_SCL);
    printf("[BB I2C]\n");
    printf("[BB I2C]      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");

    for (addr = 0; addr < 128; addr++)
    {
        if (addr % 16 == 0)
            printf("[BB I2C] %02x: ", addr);

        if (addr < 0x03 || addr > 0x77) {
            printf("   ");
        } else if (i2c_probe(addr) == 0) {
            printf("%02x ", addr);
            found++;
        } else {
            printf("-- ");
        }

        if (addr % 16 == 15)
            printf("\n");
    }

    printf("[BB I2C]\n");
    printf("[BB I2C] Found %d device(s)\n", found);
    return 0;
}

/* ============================================================
 * Test: Read from a common I2C device
 * ============================================================ */
static int test_i2c_device_read(void)
{
    uint8_t value;
    /* Common I2C addresses to try */
    uint8_t test_addrs[] = { 0x48, 0x68, 0x76, 0x3C, 0x27 };
    const char *test_names[] = {
        "TMP102/ADS1115", "MPU6050/DS3231", "BME280/BMP280",
        "SSD1306 OLED", "PCF8574 LCD"
    };
    int i, found_any = 0;

    printf("[BB I2C] Probing common I2C device addresses...\n");

    for (i = 0; i < 5; i++)
    {
        if (i2c_probe(test_addrs[i]) == 0)
        {
            printf("[BB I2C]   0x%02X (%s) -> FOUND!\n", test_addrs[i], test_names[i]);

            /* Try reading register 0x00 */
            if (i2c_read_reg(test_addrs[i], 0x00, &value) == 0)
                printf("[BB I2C]     Register 0x00 = 0x%02X\n", value);

            found_any = 1;
        }
        else
        {
            printf("[BB I2C]   0x%02X (%s) -> not found\n", test_addrs[i], test_names[i]);
        }
    }

    if (!found_any)
        printf("[BB I2C] No devices found. Connect an I2C device to SDA (pin 3) / SCL (pin 5).\n");

    return 0;
}

/* ============================================================ */

int main(int argc, char *argv[])
{
    int result = 0;
    int do_scan = (argc > 1 && argv[1][0] == 's');

    printf("=== GPIO_RPI5 I2C Test ===\n\n");

    if (gpio_init() < 0) {
        fprintf(stderr, "FATAL: gpio_init() failed. Are you on a Raspberry Pi 5?\n");
        return 1;
    }

    /* Test 1: Hardware I2C setup */
    printf("--- Test 1: Hardware I2C Pin Configuration ---\n");
    if (test_hw_i2c_setup() < 0)
        result = 1;
    test_hw_i2c_cleanup();
    printf("\n");

    /* Test 2: Bit-banged I2C - device probe */
    printf("--- Test 2: Bit-Banged I2C Device Probe ---\n");
    printf("[BB I2C] NOTE: Ensure external 4.7k pull-ups on SDA/SCL lines.\n");
    if (test_i2c_device_read() < 0)
        result = 1;
    printf("\n");

    /* Test 3: Full bus scan (optional, takes a few seconds) */
    if (do_scan)
    {
        printf("--- Test 3: Bit-Banged I2C Bus Scan ---\n");
        if (test_i2c_scan() < 0)
            result = 1;
        printf("\n");
    }
    else
    {
        printf("--- Test 3: Skipped (run with 'scan' argument for full bus scan) ---\n");
        printf("  Usage: sudo ./test_i2c scan\n\n");
    }

    /* Cleanup I2C pins */
    pinclose(I2C_SDA);
    pinclose(I2C_SCL);

    gpio_cleanup();

    printf("=== %s ===\n", result ? "SOME TESTS FAILED" : "ALL TESTS PASSED");
    return result;
}
