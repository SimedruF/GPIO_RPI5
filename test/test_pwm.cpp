/*
 * test_pwm.cpp - PWM test using GPIO_RPI5 library
 *
 * Demonstrates two PWM modes:
 *   1. Hardware PWM: configure pins to ALT0 for RP1 PWM peripheral
 *   2. Software (bit-banged) PWM: manual pulse generation via GPIO
 *
 * RPi5 PWM pin mapping (40-pin header):
 *   GPIO12 = PWM0_CHAN0 (ALT0) - Header pin 32
 *   GPIO13 = PWM0_CHAN1 (ALT0) - Header pin 33
 *   GPIO18 = PWM0_CHAN0 (ALT3) - Header pin 12
 *   GPIO19 = PWM0_CHAN1 (ALT3) - Header pin 35
 *
 * Compile:
 *   g++ test_pwm.cpp -I.. -L.. -l:gpio_rpi5.a -o test_pwm
 *
 * Run:
 *   sudo ./test_pwm
 */

#include "../gpio_rpi5.h"
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>

/* PWM pin definitions */
#define PWM0_CH0_PIN  GPIO12   /* Header pin 32 */
#define PWM0_CH1_PIN  GPIO13   /* Header pin 33 */
#define PWM_ALT0_CH0  GPIO18   /* Header pin 12 (alternate) */
#define PWM_ALT0_CH1  GPIO19   /* Header pin 35 (alternate) */

#define SW_PWM_PIN    GPIO17   /* Software PWM test pin (header pin 11) */

/* ============================================================
 * Test 1: Hardware PWM pin configuration
 * Configures PWM pins to ALT0 so RP1 PWM peripheral can drive them.
 * Actual PWM frequency/duty is controlled via /sys/class/pwm/ or
 * the RP1 PWM registers.
 * ============================================================ */
static int test_hw_pwm_setup(void)
{
    int rc = 0;

    printf("[HW PWM] Configuring PWM0 pins to ALT0...\n");

    if (pinalt(PWM0_CH0_PIN, ALT0) < 0) {
        fprintf(stderr, "[HW PWM] FAIL: Could not set GPIO%d to ALT0 (PWM0_CHAN0)\n", PWM0_CH0_PIN);
        rc = -1;
    } else {
        printf("[HW PWM]   GPIO%d -> ALT0 (PWM0_CHAN0)  OK  [Header pin 32]\n", PWM0_CH0_PIN);
    }

    if (pinalt(PWM0_CH1_PIN, ALT0) < 0) {
        fprintf(stderr, "[HW PWM] FAIL: Could not set GPIO%d to ALT0 (PWM0_CHAN1)\n", PWM0_CH1_PIN);
        rc = -1;
    } else {
        printf("[HW PWM]   GPIO%d -> ALT0 (PWM0_CHAN1)  OK  [Header pin 33]\n", PWM0_CH1_PIN);
    }

    if (rc == 0) {
        printf("[HW PWM] PWM pins configured.\n");
        printf("[HW PWM] To use hardware PWM, write to /sys/class/pwm/pwmchip0/\n");
        printf("[HW PWM]   echo 0 > /sys/class/pwm/pwmchip0/export\n");
        printf("[HW PWM]   echo 1000000 > /sys/class/pwm/pwmchip0/pwm0/period\n");
        printf("[HW PWM]   echo 500000 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle\n");
        printf("[HW PWM]   echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable\n");
    }

    return rc;
}

static void test_hw_pwm_cleanup(void)
{
    printf("[HW PWM] Restoring PWM pins to GPIO mode...\n");
    pinalt(PWM0_CH0_PIN, GPIO_FUNC_SIO);
    pinalt(PWM0_CH1_PIN, GPIO_FUNC_SIO);
    printf("[HW PWM] Done.\n");
}

/* ============================================================
 * Test 2: Software PWM (bit-banged)
 * Generates PWM signal manually by toggling GPIO.
 * Good for servos, LED dimming when HW PWM is unavailable.
 * ============================================================ */

/* Generate software PWM for a given number of cycles */
static int sw_pwm_run(int pin, int freq_hz, int duty_percent, int duration_ms)
{
    int period_us = 1000000 / freq_hz;
    int high_us = (period_us * duty_percent) / 100;
    int low_us = period_us - high_us;
    int cycles = (duration_ms * 1000) / period_us;
    int i;

    printf("[SW PWM] Pin: GPIO%d, Freq: %dHz, Duty: %d%%, Cycles: %d\n",
           pin, freq_hz, duty_percent, cycles);

    for (i = 0; i < cycles; i++) {
        pinwrite(pin, HIGH);
        usleep(high_us);
        pinwrite(pin, LOW);
        usleep(low_us);
    }

    return 0;
}

static int test_sw_pwm(void)
{
    pin_t p;

    printf("[SW PWM] Software PWM test on GPIO%d (header pin 11)\n", SW_PWM_PIN);

    p = pinopen(SW_PWM_PIN, OUTPUT);
    if (p.mode == UNDEF) {
        fprintf(stderr, "[SW PWM] FAIL: Could not open GPIO%d\n", SW_PWM_PIN);
        return -1;
    }

    /* Test various duty cycles at 1kHz for 500ms each */
    printf("[SW PWM] --- 25%% duty cycle ---\n");
    sw_pwm_run(SW_PWM_PIN, 1000, 25, 500);

    printf("[SW PWM] --- 50%% duty cycle ---\n");
    sw_pwm_run(SW_PWM_PIN, 1000, 50, 500);

    printf("[SW PWM] --- 75%% duty cycle ---\n");
    sw_pwm_run(SW_PWM_PIN, 1000, 75, 500);

    /* Servo test: 50Hz, varying duty 5-10% (1ms-2ms pulse) */
    printf("[SW PWM] --- Servo sweep (50Hz) ---\n");
    printf("[SW PWM] Servo 0 degrees (1ms pulse, 5%% duty)\n");
    sw_pwm_run(SW_PWM_PIN, 50, 5, 1000);

    printf("[SW PWM] Servo 90 degrees (1.5ms pulse, 7.5%% duty)\n");
    sw_pwm_run(SW_PWM_PIN, 50, 7, 1000);

    printf("[SW PWM] Servo 180 degrees (2ms pulse, 10%% duty)\n");
    sw_pwm_run(SW_PWM_PIN, 50, 10, 1000);

    pinwrite(SW_PWM_PIN, LOW);
    pinclose(SW_PWM_PIN);

    printf("[SW PWM] PASS: Software PWM test complete\n");
    return 0;
}

/* ============================================================
 * Test 3: Fast toggle speed measurement
 * Measures how fast pintoggle() can switch a pin.
 * ============================================================ */
static int test_toggle_speed(void)
{
    int pin = SW_PWM_PIN;
    int i;
    int toggles = 100000;

    printf("[SPEED] Toggle speed test on GPIO%d (%d toggles)\n", pin, toggles);

    pin_t p = pinopen(pin, OUTPUT);
    if (p.mode == UNDEF) {
        fprintf(stderr, "[SPEED] FAIL: Could not open GPIO%d\n", pin);
        return -1;
    }

    pinwrite(pin, LOW);

    printf("[SPEED] Toggling %d times... (measure with oscilloscope for actual frequency)\n", toggles);

    for (i = 0; i < toggles; i++) {
        pintoggle(pin);
    }

    pinwrite(pin, LOW);
    pinclose(pin);

    printf("[SPEED] PASS: %d toggles complete\n", toggles);
    printf("[SPEED] With direct register access, expect ~10-50MHz toggle rate\n");
    return 0;
}

/* ============================================================ */

int main(void)
{
    int result = 0;

    printf("=== GPIO_RPI5 PWM Test ===\n\n");

    if (gpio_init() < 0) {
        fprintf(stderr, "FATAL: gpio_init() failed. Are you on a Raspberry Pi 5?\n");
        return 1;
    }

    printf("--- Test 1: Hardware PWM Pin Configuration ---\n");
    if (test_hw_pwm_setup() < 0)
        result = 1;
    test_hw_pwm_cleanup();
    printf("\n");

    printf("--- Test 2: Software PWM ---\n");
    if (test_sw_pwm() < 0)
        result = 1;
    printf("\n");

    printf("--- Test 3: Toggle Speed ---\n");
    if (test_toggle_speed() < 0)
        result = 1;
    printf("\n");

    gpio_cleanup();

    printf("=== %s ===\n", result ? "SOME TESTS FAILED" : "ALL TESTS PASSED");
    return result;
}
