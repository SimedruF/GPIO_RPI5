/*
 * test_pcm.cpp - PCM/I2S test using GPIO_RPI5 library
 *
 * Demonstrates two PCM modes:
 *   1. Hardware PCM: configure pins to ALT0 for RP1 PCM/I2S peripheral
 *   2. Bit-banged I2S: manual I2S master TX via GPIO
 *
 * RPi5 PCM/I2S pin mapping (40-pin header):
 *   GPIO18 = PCM_CLK  (ALT0) - Header pin 12  (Bit Clock / BCLK)
 *   GPIO19 = PCM_FS   (ALT0) - Header pin 35  (Frame Sync / LRCLK)
 *   GPIO20 = PCM_DIN  (ALT0) - Header pin 38  (Data In / ADC)
 *   GPIO21 = PCM_DOUT (ALT0) - Header pin 40  (Data Out / DAC)
 *
 * I2S format (Philips standard):
 *   - BCLK: continuous clock
 *   - LRCLK: LOW = left channel, HIGH = right channel
 *   - Data: MSB first, 1 BCLK delay after LRCLK transition
 *   - 16-bit per channel, stereo = 32 BCLKs per frame
 *
 * Compile:
 *   g++ test_pcm.cpp -I.. -L.. -l:gpio_rpi5.a -o test_pcm
 *
 * Run:
 *   sudo ./test_pcm
 */

#include "../gpio_rpi5.h"
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>

/* PCM/I2S pin definitions */
#define PCM_CLK   GPIO18   /* Header pin 12 - BCLK */
#define PCM_FS    GPIO19   /* Header pin 35 - LRCLK / Word Select */
#define PCM_DIN   GPIO20   /* Header pin 38 - Data In (from ADC) */
#define PCM_DOUT  GPIO21   /* Header pin 40 - Data Out (to DAC) */

/* I2S timing */
#define I2S_BCLK_DELAY_US  1  /* Bit clock half-period (~500kHz BCLK) */

/* ============================================================
 * Test 1: Hardware PCM/I2S pin configuration
 * Sets pins to ALT0 for the RP1 PCM peripheral.
 * After this, use ALSA (aplay/arecord) for audio I/O.
 * ============================================================ */
static int test_hw_pcm_setup(void)
{
    int rc = 0;

    printf("[HW PCM] Configuring PCM/I2S pins to ALT0...\n");

    if (pinalt(PCM_CLK, ALT0) < 0) {
        fprintf(stderr, "[HW PCM] FAIL: Could not set GPIO%d to ALT0 (PCM_CLK)\n", PCM_CLK);
        rc = -1;
    } else {
        printf("[HW PCM]   GPIO%d -> ALT0 (PCM_CLK/BCLK)    OK  [Header pin 12]\n", PCM_CLK);
    }

    if (pinalt(PCM_FS, ALT0) < 0) {
        fprintf(stderr, "[HW PCM] FAIL: Could not set GPIO%d to ALT0 (PCM_FS)\n", PCM_FS);
        rc = -1;
    } else {
        printf("[HW PCM]   GPIO%d -> ALT0 (PCM_FS/LRCLK)    OK  [Header pin 35]\n", PCM_FS);
    }

    if (pinalt(PCM_DIN, ALT0) < 0) {
        fprintf(stderr, "[HW PCM] FAIL: Could not set GPIO%d to ALT0 (PCM_DIN)\n", PCM_DIN);
        rc = -1;
    } else {
        printf("[HW PCM]   GPIO%d -> ALT0 (PCM_DIN)          OK  [Header pin 38]\n", PCM_DIN);
    }

    if (pinalt(PCM_DOUT, ALT0) < 0) {
        fprintf(stderr, "[HW PCM] FAIL: Could not set GPIO%d to ALT0 (PCM_DOUT)\n", PCM_DOUT);
        rc = -1;
    } else {
        printf("[HW PCM]   GPIO%d -> ALT0 (PCM_DOUT)         OK  [Header pin 40]\n", PCM_DOUT);
    }

    /* Set drive strength for clean signals */
    pin_set_drive(PCM_CLK, DRIVE_8MA);
    pin_set_drive(PCM_FS, DRIVE_4MA);
    pin_set_drive(PCM_DOUT, DRIVE_4MA);

    if (rc == 0) {
        printf("[HW PCM] PCM/I2S pins configured.\n");
        printf("[HW PCM] To use with ALSA:\n");
        printf("[HW PCM]   Add to /boot/config.txt: dtoverlay=hifiberry-dac (or your DAC overlay)\n");
        printf("[HW PCM]   aplay -D hw:0,0 test.wav\n");
        printf("[HW PCM]   arecord -D hw:0,0 -f S16_LE -r 44100 -c 2 capture.wav\n");
    }

    return rc;
}

static void test_hw_pcm_cleanup(void)
{
    printf("[HW PCM] Restoring PCM pins to GPIO mode...\n");
    pinalt(PCM_CLK, GPIO_FUNC_SIO);
    pinalt(PCM_FS, GPIO_FUNC_SIO);
    pinalt(PCM_DIN, GPIO_FUNC_SIO);
    pinalt(PCM_DOUT, GPIO_FUNC_SIO);
    printf("[HW PCM] Done.\n");
}

/* ============================================================
 * Test 2: Bit-banged I2S master TX
 * Generates I2S frames with a sine wave pattern.
 * Useful for testing DACs without ALSA configuration.
 * ============================================================ */

static inline void bclk_delay(void)
{
    usleep(I2S_BCLK_DELAY_US);
}

/* Initialize bit-banged I2S pins */
static int bb_i2s_init(void)
{
    pin_t p;

    p = pinopen(PCM_CLK, OUTPUT);
    if (p.mode == UNDEF) return -1;
    pinwrite(PCM_CLK, LOW);

    p = pinopen(PCM_FS, OUTPUT);
    if (p.mode == UNDEF) return -1;
    pinwrite(PCM_FS, LOW);   /* Start with left channel */

    p = pinopen(PCM_DOUT, OUTPUT);
    if (p.mode == UNDEF) return -1;
    pinwrite(PCM_DOUT, LOW);

    p = pinopen(PCM_DIN, INPUT);
    if (p.mode == UNDEF) return -1;
    pinpull(PCM_DIN, PULL_DOWN);

    return 0;
}

/*
 * Transmit one I2S frame (left + right channel, 16-bit each).
 * Philips I2S standard:
 *   - LRCLK transitions 1 BCLK before MSB
 *   - Data is MSB first
 *   - Data valid on rising edge of BCLK
 */
static void bb_i2s_tx_frame(int16_t left, int16_t right)
{
    int i;
    uint16_t data;

    /* --- Left channel (LRCLK = LOW) --- */
    /* LRCLK transition happens 1 BCLK before MSB */
    pinwrite(PCM_FS, LOW);
    data = (uint16_t)left;

    for (i = 15; i >= 0; i--)
    {
        /* Falling edge: set data */
        pinwrite(PCM_CLK, LOW);
        pinwrite(PCM_DOUT, (data >> i) & 1);
        bclk_delay();

        /* Rising edge: receiver samples data */
        pinwrite(PCM_CLK, HIGH);
        bclk_delay();
    }

    /* --- Right channel (LRCLK = HIGH) --- */
    pinwrite(PCM_FS, HIGH);
    data = (uint16_t)right;

    for (i = 15; i >= 0; i--)
    {
        pinwrite(PCM_CLK, LOW);
        pinwrite(PCM_DOUT, (data >> i) & 1);
        bclk_delay();

        pinwrite(PCM_CLK, HIGH);
        bclk_delay();
    }
}

/* Read one I2S frame from DIN (16-bit left + right) */
static void bb_i2s_rx_frame(int16_t *left, int16_t *right)
{
    int i;
    uint16_t data;

    /* Left channel */
    pinwrite(PCM_FS, LOW);
    data = 0;
    for (i = 15; i >= 0; i--)
    {
        pinwrite(PCM_CLK, LOW);
        bclk_delay();
        pinwrite(PCM_CLK, HIGH);
        bclk_delay();
        if (pinread(PCM_DIN))
            data |= (1 << i);
    }
    *left = (int16_t)data;

    /* Right channel */
    pinwrite(PCM_FS, HIGH);
    data = 0;
    for (i = 15; i >= 0; i--)
    {
        pinwrite(PCM_CLK, LOW);
        bclk_delay();
        pinwrite(PCM_CLK, HIGH);
        bclk_delay();
        if (pinread(PCM_DIN))
            data |= (1 << i);
    }
    *right = (int16_t)data;
}

static void bb_i2s_cleanup(void)
{
    pinclose(PCM_CLK);
    pinclose(PCM_FS);
    pinclose(PCM_DOUT);
    pinclose(PCM_DIN);
}

/* ============================================================
 * Test: Generate sine wave I2S frames
 * ============================================================ */
static int test_bb_i2s_sine(void)
{
    int i;
    int frames = 256;  /* One cycle of a sine wave at ~256 samples */
    int16_t left, right;

    printf("[BB I2S] Bit-banged I2S sine wave test\n");
    printf("[BB I2S] Pins: BCLK=GPIO%d, LRCLK=GPIO%d, DOUT=GPIO%d, DIN=GPIO%d\n",
           PCM_CLK, PCM_FS, PCM_DOUT, PCM_DIN);
    printf("[BB I2S] Format: 16-bit stereo, Philips I2S standard\n");
    printf("[BB I2S] Generating %d frames of sine wave...\n", frames);

    if (bb_i2s_init() < 0) {
        fprintf(stderr, "[BB I2S] FAIL: Could not initialize I2S pins\n");
        return -1;
    }

    for (i = 0; i < frames; i++)
    {
        /* Generate 16-bit sine wave samples */
        double phase = (2.0 * 3.14159265 * i) / frames;
        left  = (int16_t)(sin(phase) * 16000);
        right = (int16_t)(sin(phase + 1.57) * 16000);  /* 90 degrees offset */

        bb_i2s_tx_frame(left, right);
    }

    /* Silence after test */
    bb_i2s_tx_frame(0, 0);

    printf("[BB I2S] Sine wave generation complete.\n");
    printf("[BB I2S] Connect an I2S DAC to hear audio or use oscilloscope to verify.\n");

    bb_i2s_cleanup();

    printf("[BB I2S] PASS\n");
    return 0;
}

/* ============================================================
 * Test: I2S clock and frame sync pattern
 * Generates a fixed pattern for easy oscilloscope verification.
 * ============================================================ */
static int test_bb_i2s_pattern(void)
{
    int i;

    printf("[BB I2S] I2S clock/data pattern test\n");
    printf("[BB I2S] Sending alternating 0xAAAA / 0x5555 pattern (10 frames)\n");

    if (bb_i2s_init() < 0) {
        fprintf(stderr, "[BB I2S] FAIL: Could not initialize I2S pins\n");
        return -1;
    }

    for (i = 0; i < 10; i++)
    {
        bb_i2s_tx_frame(0x5555, (int16_t)0xAAAA);
    }

    /* Verify pin states after transmission */
    printf("[BB I2S] Pin states after transmission:\n");
    printf("[BB I2S]   BCLK  (GPIO%d) = %s\n", PCM_CLK,
           pinread(PCM_CLK) ? "HIGH" : "LOW");
    printf("[BB I2S]   LRCLK (GPIO%d) = %s\n", PCM_FS,
           pinread(PCM_FS) ? "HIGH (right ch)" : "LOW (left ch)");

    bb_i2s_cleanup();

    printf("[BB I2S] PASS: Pattern test complete\n");
    return 0;
}

/* ============================================================
 * Test: Read from I2S ADC (DIN pin)
 * ============================================================ */
static int test_bb_i2s_read(void)
{
    int i;
    int16_t left, right;

    printf("[BB I2S] I2S read test (DIN on GPIO%d, pin 38)\n", PCM_DIN);
    printf("[BB I2S] NOTE: Connect an I2S ADC (e.g., INMP441 mic) for real data.\n");
    printf("[BB I2S] Reading 8 frames...\n");

    if (bb_i2s_init() < 0) {
        fprintf(stderr, "[BB I2S] FAIL: Could not initialize I2S pins\n");
        return -1;
    }

    for (i = 0; i < 8; i++)
    {
        bb_i2s_rx_frame(&left, &right);
        printf("[BB I2S]   Frame %d: L=0x%04X (%6d)  R=0x%04X (%6d)\n",
               i, (uint16_t)left, left, (uint16_t)right, right);
    }

    bb_i2s_cleanup();

    printf("[BB I2S] PASS: Read test complete\n");
    return 0;
}

/* ============================================================ */

int main(void)
{
    int result = 0;

    printf("=== GPIO_RPI5 PCM/I2S Test ===\n\n");

    if (gpio_init() < 0) {
        fprintf(stderr, "FATAL: gpio_init() failed. Are you on a Raspberry Pi 5?\n");
        return 1;
    }

    /* Test 1: Hardware PCM setup */
    printf("--- Test 1: Hardware PCM/I2S Pin Configuration ---\n");
    if (test_hw_pcm_setup() < 0)
        result = 1;
    test_hw_pcm_cleanup();
    printf("\n");

    /* Test 2: I2S clock/data pattern */
    printf("--- Test 2: Bit-Banged I2S Pattern ---\n");
    if (test_bb_i2s_pattern() < 0)
        result = 1;
    printf("\n");

    /* Test 3: I2S sine wave generation */
    printf("--- Test 3: Bit-Banged I2S Sine Wave ---\n");
    if (test_bb_i2s_sine() < 0)
        result = 1;
    printf("\n");

    /* Test 4: I2S read from DIN */
    printf("--- Test 4: Bit-Banged I2S Read ---\n");
    if (test_bb_i2s_read() < 0)
        result = 1;
    printf("\n");

    gpio_cleanup();

    printf("=== %s ===\n", result ? "SOME TESTS FAILED" : "ALL TESTS PASSED");
    return result;
}
