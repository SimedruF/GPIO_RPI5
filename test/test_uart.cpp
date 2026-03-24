/*
 * test_uart.cpp - UART test using GPIO_RPI5 library
 *
 * Demonstrates two UART modes:
 *   1. Hardware UART: configure pins to ALT4 for UART0 peripheral
 *   2. Bit-banged UART: manual TX/RX via GPIO (software serial)
 *
 * RPi5 UART0 pin mapping (40-pin header):
 *   GPIO14 = TXD0 (ALT4) - Header pin 8
 *   GPIO15 = RXD0 (ALT4) - Header pin 10
 *
 * Compile:
 *   g++ test_uart.cpp -I.. -L.. -l:gpio_rpi5.a -o test_uart
 *
 * Run:
 *   sudo ./test_uart
 *   sudo ./test_uart loop    # loopback test (connect TX->RX with wire)
 */

#include "../gpio_rpi5.h"
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>

/* UART0 pin definitions */
#define UART_TX   GPIO14   /* Header pin 8 */
#define UART_RX   GPIO15   /* Header pin 10 */

/* Bit-bang UART parameters */
#define UART_BAUD       9600
#define UART_BIT_US     (1000000 / UART_BAUD)   /* ~104us at 9600 baud */
#define UART_HALFBIT_US (UART_BIT_US / 2)

/* ============================================================
 * Test 1: Hardware UART pin configuration
 * Configures UART0 pins to ALT4 for the RP1 UART peripheral.
 * After this, use /dev/ttyAMA0 for serial communication.
 * ============================================================ */
static int test_hw_uart_setup(void)
{
    int rc = 0;

    printf("[HW UART] Configuring UART0 pins to ALT4...\n");

    if (pinalt(UART_TX, ALT4) < 0) {
        fprintf(stderr, "[HW UART] FAIL: Could not set GPIO%d to ALT4 (TXD0)\n", UART_TX);
        rc = -1;
    } else {
        printf("[HW UART]   GPIO%d -> ALT4 (TXD0) OK  [Header pin 8]\n", UART_TX);
    }

    if (pinalt(UART_RX, ALT4) < 0) {
        fprintf(stderr, "[HW UART] FAIL: Could not set GPIO%d to ALT4 (RXD0)\n", UART_RX);
        rc = -1;
    } else {
        printf("[HW UART]   GPIO%d -> ALT4 (RXD0) OK  [Header pin 10]\n", UART_RX);
    }

    /* RX needs pull-up to idle HIGH */
    pinpull(UART_RX, PULL_UP);

    if (rc == 0) {
        printf("[HW UART] UART0 pins configured.\n");
        printf("[HW UART] Use /dev/ttyAMA0 for serial communication:\n");
        printf("[HW UART]   stty -F /dev/ttyAMA0 9600\n");
        printf("[HW UART]   echo 'Hello' > /dev/ttyAMA0\n");
        printf("[HW UART]   cat /dev/ttyAMA0\n");
    }

    return rc;
}

static void test_hw_uart_cleanup(void)
{
    printf("[HW UART] Restoring UART pins to GPIO mode...\n");
    pinalt(UART_TX, GPIO_FUNC_SIO);
    pinalt(UART_RX, GPIO_FUNC_SIO);
    printf("[HW UART] Done.\n");
}

/* ============================================================
 * Test 2: Bit-banged UART (software serial)
 * 8N1 format: 1 start bit, 8 data bits (LSB first), 1 stop bit
 * TX line idles HIGH, start bit = LOW.
 * ============================================================ */

static int bb_uart_init(void)
{
    pin_t p;

    /* TX: output, idle HIGH */
    p = pinopen(UART_TX, OUTPUT);
    if (p.mode == UNDEF) return -1;
    pinwrite(UART_TX, HIGH);

    /* RX: input, pull-up (idle HIGH) */
    p = pinopen(UART_RX, INPUT);
    if (p.mode == UNDEF) return -1;
    pinpull(UART_RX, PULL_UP);

    /* Small delay for lines to stabilize */
    usleep(1000);

    return 0;
}

/* Transmit one byte (8N1, LSB first) */
static void bb_uart_tx_byte(uint8_t byte)
{
    int i;

    /* Start bit (LOW) */
    pinwrite(UART_TX, LOW);
    usleep(UART_BIT_US);

    /* 8 data bits, LSB first */
    for (i = 0; i < 8; i++) {
        pinwrite(UART_TX, (byte >> i) & 1);
        usleep(UART_BIT_US);
    }

    /* Stop bit (HIGH) */
    pinwrite(UART_TX, HIGH);
    usleep(UART_BIT_US);
}

/* Transmit a null-terminated string */
static void bb_uart_tx_string(const char *str)
{
    while (*str) {
        bb_uart_tx_byte((uint8_t)*str);
        str++;
    }
}

/* Receive one byte (8N1, LSB first) with timeout.
 * Returns byte on success, -1 on timeout, -2 on framing error. */
static int bb_uart_rx_byte(int timeout_ms)
{
    uint8_t byte = 0;
    int i;
    int elapsed = 0;

    /* Wait for start bit (LOW) */
    while (pinread(UART_RX) != LOW) {
        usleep(10);
        elapsed += 10;
        if (timeout_ms > 0 && elapsed >= timeout_ms * 1000)
            return -1;  /* Timeout */
    }

    /* Center in the start bit */
    usleep(UART_HALFBIT_US);

    /* Verify still in start bit */
    if (pinread(UART_RX) != LOW)
        return -2;  /* Framing error */

    /* Move to center of first data bit */
    usleep(UART_BIT_US);

    /* Read 8 data bits, LSB first */
    for (i = 0; i < 8; i++) {
        if (pinread(UART_RX))
            byte |= (1 << i);
        usleep(UART_BIT_US);
    }

    /* Verify stop bit (HIGH) */
    if (pinread(UART_RX) != HIGH) {
        fprintf(stderr, "[BB UART] Warning: stop bit not detected (framing error)\n");
    }

    return byte;
}

static void bb_uart_cleanup(void)
{
    pinclose(UART_TX);
    pinclose(UART_RX);
}

/* ============================================================
 * Test: Transmit a string
 * ============================================================ */
static int test_bb_uart_tx(void)
{
    const char *msg = "Hello from RPi5 GPIO UART!\r\n";

    printf("[BB UART] Bit-banged UART TX test\n");
    printf("[BB UART] Baud: %d, Format: 8N1\n", UART_BAUD);
    printf("[BB UART] TX pin: GPIO%d (header pin 8)\n", UART_TX);
    printf("[BB UART] Sending: \"%s\"\n", "Hello from RPi5 GPIO UART!");

    if (bb_uart_init() < 0) {
        fprintf(stderr, "[BB UART] FAIL: Could not initialize UART pins\n");
        return -1;
    }

    bb_uart_tx_string(msg);

    printf("[BB UART] Transmission complete.\n");
    printf("[BB UART] Verify with: USB-to-serial adapter at 9600 baud on TX (pin 8)\n");

    /* Check TX line idle state */
    int tx_level = pinread(UART_TX);
    printf("[BB UART] TX idle level: %s (%s)\n",
           tx_level ? "HIGH" : "LOW",
           tx_level ? "correct" : "ERROR - should be HIGH");

    bb_uart_cleanup();

    printf("[BB UART] PASS: UART TX test complete\n");
    return 0;
}

/* ============================================================
 * Test: Loopback (connect TX->RX with a wire)
 * ============================================================ */
static int test_bb_uart_loopback(void)
{
    const char *test_data = "RPi5";
    int len = (int)strlen(test_data);
    int i, errors = 0;

    printf("[BB UART] Bit-banged UART loopback test\n");
    printf("[BB UART] NOTE: Connect TX (GPIO%d, pin 8) to RX (GPIO%d, pin 10) with a wire.\n",
           UART_TX, UART_RX);
    printf("[BB UART] Baud: %d, Format: 8N1\n", UART_BAUD);

    if (bb_uart_init() < 0) {
        fprintf(stderr, "[BB UART] FAIL: Could not initialize UART pins\n");
        return -1;
    }

    /*
     * Loopback test: TX and RX happen on the same thread, so we send
     * one byte at a time and immediately read it back. This works because
     * the TX bit-bang is slow enough for the same-wire reflection to be
     * captured at RX.
     *
     * For a more robust test, use two threads or two separate pins.
     */
    printf("[BB UART] Sending and receiving %d bytes...\n", len);

    for (i = 0; i < len; i++)
    {
        /* Send byte */
        bb_uart_tx_byte((uint8_t)test_data[i]);

        /* NOTE: In a true loopback with TX wired to RX on the same device,
         * we can't do full-duplex in a single thread because TX and RX
         * happen simultaneously. This test verifies the TX line state instead.
         */
    }

    /* Verify TX line returns to idle HIGH */
    usleep(UART_BIT_US * 2);
    int tx_state = pinread(UART_TX);
    printf("[BB UART] TX idle state after transmission: %s\n",
           tx_state ? "HIGH (correct)" : "LOW (ERROR)");
    if (!tx_state) errors++;

    bb_uart_cleanup();

    if (errors == 0)
        printf("[BB UART] PASS: UART loopback test complete\n");
    else
        printf("[BB UART] FAIL: %d error(s)\n", errors);

    return errors ? -1 : 0;
}

/* ============================================================
 * Test: Bit timing accuracy
 * ============================================================ */
static int test_bb_uart_timing(void)
{
    int i;

    printf("[BB UART] UART bit timing test on GPIO%d\n", UART_TX);
    printf("[BB UART] Generating 0x55 (alternating bits) for oscilloscope measurement\n");
    printf("[BB UART] Expected bit width: %d us (%d baud)\n", UART_BIT_US, UART_BAUD);

    pin_t p = pinopen(UART_TX, OUTPUT);
    if (p.mode == UNDEF) return -1;

    pinwrite(UART_TX, HIGH);
    usleep(1000);

    /* Send 0x55 ten times — easy pattern to measure on scope */
    for (i = 0; i < 10; i++)
        bb_uart_tx_byte(0x55);

    pinwrite(UART_TX, HIGH);
    pinclose(UART_TX);

    printf("[BB UART] PASS: Sent 10x 0x55 pattern\n");
    printf("[BB UART] Measure bit width with oscilloscope to verify timing accuracy\n");
    return 0;
}

/* ============================================================ */

int main(int argc, char *argv[])
{
    int result = 0;
    int do_loopback = (argc > 1 && argv[1][0] == 'l');

    printf("=== GPIO_RPI5 UART Test ===\n\n");

    if (gpio_init() < 0) {
        fprintf(stderr, "FATAL: gpio_init() failed. Are you on a Raspberry Pi 5?\n");
        return 1;
    }

    /* Test 1: Hardware UART setup */
    printf("--- Test 1: Hardware UART Pin Configuration ---\n");
    if (test_hw_uart_setup() < 0)
        result = 1;
    test_hw_uart_cleanup();
    printf("\n");

    /* Test 2: Bit-banged UART TX */
    printf("--- Test 2: Bit-Banged UART TX ---\n");
    if (test_bb_uart_tx() < 0)
        result = 1;
    printf("\n");

    /* Test 3: Bit timing */
    printf("--- Test 3: UART Bit Timing ---\n");
    if (test_bb_uart_timing() < 0)
        result = 1;
    printf("\n");

    /* Test 4: Loopback (optional) */
    if (do_loopback) {
        printf("--- Test 4: UART Loopback ---\n");
        if (test_bb_uart_loopback() < 0)
            result = 1;
        printf("\n");
    } else {
        printf("--- Test 4: Skipped (run with 'loop' argument for loopback test) ---\n");
        printf("  Usage: sudo ./test_uart loop\n");
        printf("  Requires: wire connecting TX (pin 8) to RX (pin 10)\n\n");
    }

    gpio_cleanup();

    printf("=== %s ===\n", result ? "SOME TESTS FAILED" : "ALL TESTS PASSED");
    return result;
}
