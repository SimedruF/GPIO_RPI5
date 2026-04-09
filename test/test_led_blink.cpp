/**
 * @file test_led_blink.cpp
 * @brief LED blink test using the gpio_rpi5 mmap backend.
 *
 * Wiring (40-pin header):
 *   Pin 11 (GPIO17) ──── LED(+) ──── R(330Ω) ──── Pin 6 (GND)
 *
 * Build:  ./test/build_led_blink.sh
 * Run:    sudo ./test/test_led_blink
 */

#include "../gpio_rpi5.h"
#include <stdio.h>
#include <unistd.h>  /* usleep, sleep */
#include <signal.h>

/* ---- Configuration ---- */
#define LED_PIN     GPIO17   /* Physical pin 11 — free at boot */
#define BLINK_COUNT 10       /* Number of on/off cycles */
#define BLINK_MS    500      /* Half-period in milliseconds */

static volatile int running = 1;

static void sigint_handler(int sig)
{
  (void)sig;
  running = 0;
}

int main(void)
{
  int i;

  /* Catch Ctrl-C so we always clean up */
  signal(SIGINT, sigint_handler);

  printf("=== LED Blink Test ===\n");
  printf("Pin:   GPIO%d (physical pin 11)\n", LED_PIN);
  printf("Blink: %d cycles, %d ms half-period\n\n", BLINK_COUNT, BLINK_MS);

  /* 1. Initialise GPIO subsystem */
  if (gpio_init() < 0)
  {
    fprintf(stderr, "gpio_init() failed. Are you on a RPi5? Try: sudo ./test_led_blink\n");
    return 1;
  }
  printf("[OK] gpio_init()\n");

  /* 2. Open pin as output */
  pin_t led = pinopen(LED_PIN, OUTPUT);
  if (led.mode == UNDEF)
  {
    fprintf(stderr, "pinopen(GPIO%d, OUTPUT) failed\n", LED_PIN);
    gpio_cleanup();
    return 1;
  }
  printf("[OK] pinopen(GPIO%d, OUTPUT)\n", LED_PIN);

  /* 3. Blink loop */
  printf("\nBlinking LED ...\n");
  for (i = 0; i < BLINK_COUNT && running; i++)
  {
    pinwrite(LED_PIN, HIGH);
    printf("  [%2d] HIGH\n", i + 1);
    usleep(BLINK_MS * 1000);

    pinwrite(LED_PIN, LOW);
    printf("  [%2d] LOW\n", i + 1);
    usleep(BLINK_MS * 1000);
  }

  /* 4. Bonus: fast toggle using pintoggle() */
  if (running)
  {
    printf("\nFast toggle demo (5 toggles, 100 ms each)...\n");
    for (i = 0; i < 5 && running; i++)
    {
      pintoggle(LED_PIN);
      printf("  toggle %d -> read=%d\n", i + 1, pinread(LED_PIN));
      usleep(100 * 1000);
    }
  }

  /* 5. Clean up — always turn LED off and release the pin */
  printf("\nCleaning up...\n");
  pinwrite(LED_PIN, LOW);
  pinclose(LED_PIN);
  gpio_cleanup();
  printf("[OK] Done.\n");

  return 0;
}
