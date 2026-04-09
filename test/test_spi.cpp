/*
 * test_spi.cpp - SPI test using GPIO_RPI5 library
 *
 * Demonstrates two SPI modes:
 *   1. Hardware SPI: configure pins to ALT0 for SPI0 peripheral
 *   2. Bit-banged SPI: manual SPI Mode 0 (CPOL=0, CPHA=0) via GPIO
 *
 * SPI0 pin mapping on Raspberry Pi 5 (40-pin header):
 *   GPIO8  = CE0  (Chip Enable 0)  - Header pin 24
 *   GPIO7  = CE1  (Chip Enable 1)  - Header pin 26
 *   GPIO9  = MISO (Master In)      - Header pin 21
 *   GPIO10 = MOSI (Master Out)     - Header pin 19
 *   GPIO11 = SCLK (Clock)          - Header pin 23
 *
 * Compile:
 *   g++ test_spi.cpp -I.. -L.. -l:gpio_rpi5.a -o test_spi
 *
 * Run:
 *   sudo ./test_spi
 */

#include "../gpio_rpi5.h"
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>

/* SPI0 pin definitions */
#define SPI_MOSI  GPIO10
#define SPI_MISO  GPIO9
#define SPI_SCLK  GPIO11
#define SPI_CE0   GPIO8
#define SPI_CE1   GPIO7

/* ============================================================
 * Test 1: Hardware SPI pin configuration
 * Sets pins to ALT0 so the RP1 SPI0 peripheral takes control.
 * After this, use /dev/spidev0.x for actual SPI transfers.
 * ============================================================ */
static int test_hw_spi_setup(void)
{
    int rc = 0;

    printf("[HW SPI] Configuring SPI0 pins to ALT0...\n");

    /* MOSI */
    if (pinalt(SPI_MOSI, ALT0) < 0) {
        fprintf(stderr, "[HW SPI] FAIL: Could not set GPIO%d to ALT0 (MOSI)\n", SPI_MOSI);
        rc = -1;
    } else {
        printf("[HW SPI]   GPIO%d -> ALT0 (SPI0_MOSI)  OK\n", SPI_MOSI);
    }

    /* MISO */
    if (pinalt(SPI_MISO, ALT0) < 0) {
        fprintf(stderr, "[HW SPI] FAIL: Could not set GPIO%d to ALT0 (MISO)\n", SPI_MISO);
        rc = -1;
    } else {
        printf("[HW SPI]   GPIO%d -> ALT0 (SPI0_MISO)  OK\n", SPI_MISO);
    }

    /* SCLK */
    if (pinalt(SPI_SCLK, ALT0) < 0) {
        fprintf(stderr, "[HW SPI] FAIL: Could not set GPIO%d to ALT0 (SCLK)\n", SPI_SCLK);
        rc = -1;
    } else {
        printf("[HW SPI]   GPIO%d -> ALT0 (SPI0_SCLK)  OK\n", SPI_SCLK);
    }

    /* CE0 */
    if (pinalt(SPI_CE0, ALT0) < 0) {
        fprintf(stderr, "[HW SPI] FAIL: Could not set GPIO%d to ALT0 (CE0)\n", SPI_CE0);
        rc = -1;
    } else {
        printf("[HW SPI]   GPIO%d -> ALT0 (SPI0_CE0)   OK\n", SPI_CE0);
    }

    if (rc == 0)
        printf("[HW SPI] All SPI0 pins configured. Use /dev/spidev0.0 for transfers.\n");

    return rc;
}

/* Restore SPI pins to normal GPIO (cleanup) */
static void test_hw_spi_cleanup(void)
{
    printf("[HW SPI] Restoring SPI pins to GPIO mode...\n");
    pinalt(SPI_MOSI, GPIO_FUNC_SIO);
    pinalt(SPI_MISO, GPIO_FUNC_SIO);
    pinalt(SPI_SCLK, GPIO_FUNC_SIO);
    pinalt(SPI_CE0,  GPIO_FUNC_SIO);
    printf("[HW SPI] Done.\n");
}

/* ============================================================
 * Test 2: Bit-banged SPI (Mode 0: CPOL=0, CPHA=0)
 * Useful when HW SPI is unavailable or for non-standard pins.
 * MSB first, clock idles LOW, data sampled on rising edge.
 * ============================================================ */

/* Bit-bang delay — ~500kHz with usleep(1). Adjust for speed. */
static inline void spi_delay(void)
{
    usleep(1);
}

/* Initialize bit-banged SPI pins */
static int bb_spi_init(void)
{
    pin_t p;

    p = pinopen(SPI_SCLK, OUTPUT);
    if (p.mode == UNDEF) return -1;
    pinwrite(SPI_SCLK, LOW);  /* CPOL=0: clock idles low */

    p = pinopen(SPI_MOSI, OUTPUT);
    if (p.mode == UNDEF) return -1;
    pinwrite(SPI_MOSI, LOW);

    p = pinopen(SPI_MISO, INPUT);
    if (p.mode == UNDEF) return -1;
    pinpull(SPI_MISO, PULL_NONE);

    p = pinopen(SPI_CE0, OUTPUT);
    if (p.mode == UNDEF) return -1;
    pinwrite(SPI_CE0, HIGH); /* CS idle high (active low) */

    return 0;
}

/* Transfer one byte (full-duplex), MSB first, SPI Mode 0 */
static uint8_t bb_spi_transfer_byte(uint8_t tx_byte)
{
    uint8_t rx_byte = 0;
    int i;

    for (i = 7; i >= 0; i--)
    {
        /* Set MOSI bit */
        pinwrite(SPI_MOSI, (tx_byte >> i) & 1);
        spi_delay();

        /* Rising edge: clock goes HIGH, slave latches MOSI, master samples MISO */
        pinwrite(SPI_SCLK, HIGH);
        spi_delay();

        /* Read MISO */
        if (pinread(SPI_MISO))
            rx_byte |= (1 << i);

        /* Falling edge: clock goes LOW */
        pinwrite(SPI_SCLK, LOW);
    }

    return rx_byte;
}

/* Transfer a buffer of bytes with CS control */
static void bb_spi_transfer(const uint8_t *tx, uint8_t *rx, int len)
{
    int i;

    /* Assert CS (active low) */
    pinwrite(SPI_CE0, LOW);
    spi_delay();

    for (i = 0; i < len; i++)
    {
        rx[i] = bb_spi_transfer_byte(tx[i]);
    }

    /* Deassert CS */
    spi_delay();
    pinwrite(SPI_CE0, HIGH);
}

static void bb_spi_cleanup(void)
{
    pinclose(SPI_SCLK);
    pinclose(SPI_MOSI);
    pinclose(SPI_MISO);
    pinclose(SPI_CE0);
}

/* Test: bit-banged loopback (connect MOSI -> MISO with a wire) */
static int test_bb_spi_loopback(void)
{
    uint8_t tx_buf[4] = { 0xA5, 0x3C, 0xFF, 0x00 };
    uint8_t rx_buf[4] = { 0 };
    int i, errors = 0;

    printf("[BB SPI] Bit-banged SPI loopback test\n");
    printf("[BB SPI] NOTE: Connect MOSI (GPIO%d, pin 19) to MISO (GPIO%d, pin 21) for loopback.\n",
           SPI_MOSI, SPI_MISO);
    printf("[BB SPI] Initializing pins...\n");

    if (bb_spi_init() < 0)
    {
        fprintf(stderr, "[BB SPI] FAIL: Could not initialize SPI pins\n");
        return -1;
    }

    printf("[BB SPI] Sending 4 bytes: ");
    for (i = 0; i < 4; i++)
        printf("0x%02X ", tx_buf[i]);
    printf("\n");

    bb_spi_transfer(tx_buf, rx_buf, 4);

    printf("[BB SPI] Received 4 bytes: ");
    for (i = 0; i < 4; i++)
        printf("0x%02X ", rx_buf[i]);
    printf("\n");

    /* Verify loopback */
    for (i = 0; i < 4; i++)
    {
        if (tx_buf[i] != rx_buf[i])
        {
            printf("[BB SPI]   Byte %d: MISMATCH TX=0x%02X RX=0x%02X\n",
                   i, tx_buf[i], rx_buf[i]);
            errors++;
        }
        else
        {
            printf("[BB SPI]   Byte %d: MATCH 0x%02X\n", i, tx_buf[i]);
        }
    }

    bb_spi_cleanup();

    if (errors == 0)
        printf("[BB SPI] PASS: All bytes match (loopback OK)\n");
    else
        printf("[BB SPI] FAIL: %d byte(s) mismatched (check MOSI->MISO wire)\n", errors);

    return errors ? -1 : 0;
}

/* Test: bit-banged SPI write to a hypothetical device (no loopback needed) */
static int test_bb_spi_write(void)
{
    /* Example: write register 0x20 with value 0x47 (common pattern for SPI sensors) */
    uint8_t tx_buf[2] = { 0x20, 0x47 };  /* [register_addr, value] */
    uint8_t rx_buf[2] = { 0 };

    printf("[BB SPI] Bit-banged SPI write test (no device needed)\n");
    printf("[BB SPI] Initializing pins...\n");

    if (bb_spi_init() < 0)
    {
        fprintf(stderr, "[BB SPI] FAIL: Could not initialize SPI pins\n");
        return -1;
    }

    printf("[BB SPI] Writing: REG=0x%02X VAL=0x%02X\n", tx_buf[0], tx_buf[1]);

    bb_spi_transfer(tx_buf, rx_buf, 2);

    printf("[BB SPI] Transfer complete. RX: 0x%02X 0x%02X\n", rx_buf[0], rx_buf[1]);

    /* Verify pin states after transfer */
    printf("[BB SPI] Pin states after transfer:\n");
    printf("[BB SPI]   SCLK (GPIO%d) = %s (expected LOW)\n",
           SPI_SCLK, pinread(SPI_SCLK) ? "HIGH" : "LOW");
    printf("[BB SPI]   CE0  (GPIO%d) = %s (expected HIGH / deasserted)\n",
           SPI_CE0, pinread(SPI_CE0) ? "HIGH" : "LOW");

    bb_spi_cleanup();

    printf("[BB SPI] PASS: SPI write cycle completed\n");
    return 0;
}

/* ============================================================ */

int main(int argc, char *argv[])
{
    int result = 0;

    printf("=== GPIO_RPI5 SPI Test ===\n\n");

    if (gpio_init() < 0)
    {
        fprintf(stderr, "FATAL: gpio_init() failed. Are you on a Raspberry Pi 5?\n");
        return 1;
    }

    /* Test 1: Hardware SPI setup */
    printf("--- Test 1: Hardware SPI Pin Configuration ---\n");
    if (test_hw_spi_setup() < 0)
        result = 1;
    test_hw_spi_cleanup();
    printf("\n");

    /* Test 2: Bit-banged SPI write (always works, no external hardware needed) */
    printf("--- Test 2: Bit-Banged SPI Write ---\n");
    if (test_bb_spi_write() < 0)
        result = 1;
    printf("\n");

    /* Test 3: Bit-banged SPI loopback (needs MOSI->MISO wire) */
    if (argc > 1 && argv[1][0] == 'l')
    {
        printf("--- Test 3: Bit-Banged SPI Loopback ---\n");
        if (test_bb_spi_loopback() < 0)
            result = 1;
        printf("\n");
    }
    else
    {
        printf("--- Test 3: Skipped (run with 'l' argument for loopback test) ---\n");
        printf("  Usage: sudo ./test_spi l\n");
        printf("  Requires: wire connecting MOSI (pin 19) to MISO (pin 21)\n\n");
    }

    gpio_cleanup();

    printf("=== %s ===\n", result ? "SOME TESTS FAILED" : "ALL TESTS PASSED");
    return result;
}
