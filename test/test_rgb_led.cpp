/**
 * @file test_rgb_led.cpp
 * @brief RGB LED test using the gpio_rpi5 mmap backend.
 *
 * Drives a common-cathode RGB LED through 7 colour combinations,
 * then runs a chase/cycle animation. Each colour channel is on a
 * separate GPIO pin.
 *
 * @par Wiring (40-pin header, common-cathode RGB LED)
 *
 *   Pin 11 (GPIO17) ── R(330Ω) ── RED   anode
 *   Pin 13 (GPIO27) ── R(330Ω) ── GREEN anode
 *   Pin 15 (GPIO22) ── R(330Ω) ── BLUE  anode
 *   Pin 6  (GND)    ── common cathode (-)
 *
 *   For a common-ANODE LED, connect the common pin to 3.3 V (Pin 1)
 *   and invert the logic (HIGH = off, LOW = on).
 *
 * Build:  ./test/build_rgb_led.sh
 * Run:    sudo ./test/test_rgb_led
 */

#include "../gpio_rpi5.h"
#include <stdio.h>
#include <unistd.h>   /* usleep */
#include <signal.h>
#include <string.h>

/* ---- Configuration ---- */
#define PIN_RED     GPIO17   /* Physical pin 11 */
#define PIN_GREEN   GPIO27   /* Physical pin 13 */
#define PIN_BLUE    GPIO22   /* Physical pin 15 */

#define COLOUR_HOLD_MS  800  /* How long each colour is displayed */
#define CYCLE_MS        200  /* Speed of the chase animation */
#define CYCLE_ROUNDS    3    /* Number of full chase cycles */

/* Set to 1 if your LED is common-anode (active LOW) */
#define COMMON_ANODE    0

static volatile int running = 1;

static void sigint_handler(int sig)
{
  (void)sig;
  running = 0;
}

/* ---- Helpers ---- */

/** @brief Write a colour to the RGB LED (r, g, b: 0 or 1). */
static void rgb_set(int r, int g, int b)
{
#if COMMON_ANODE
  /* Invert for common-anode: HIGH = off, LOW = on */
  pinwrite(PIN_RED,   r ? LOW : HIGH);
  pinwrite(PIN_GREEN, g ? LOW : HIGH);
  pinwrite(PIN_BLUE,  b ? LOW : HIGH);
#else
  pinwrite(PIN_RED,   r ? HIGH : LOW);
  pinwrite(PIN_GREEN, g ? HIGH : LOW);
  pinwrite(PIN_BLUE,  b ? HIGH : LOW);
#endif
}

/** @brief Turn all channels off. */
static void rgb_off(void)
{
  rgb_set(0, 0, 0);
}

/** @brief Colour descriptor for the test sequence. */
typedef struct {
  const char *name;
  int r, g, b;
} colour_t;

/* 7 non-black combinations */
static const colour_t colours[] = {
  { "RED",     1, 0, 0 },
  { "GREEN",   0, 1, 0 },
  { "BLUE",    0, 0, 1 },
  { "YELLOW",  1, 1, 0 },
  { "CYAN",    0, 1, 1 },
  { "MAGENTA", 1, 0, 1 },
  { "WHITE",   1, 1, 1 },
};
#define NUM_COLOURS (int)(sizeof(colours) / sizeof(colours[0]))

/* ---- Main ---- */

int main(void)
{
  int i, round;

  signal(SIGINT, sigint_handler);

  printf("=== RGB LED Test ===\n");
  printf("Pins:  RED=GPIO%d (pin 11)  GREEN=GPIO%d (pin 13)  BLUE=GPIO%d (pin 15)\n",
         PIN_RED, PIN_GREEN, PIN_BLUE);
  printf("Mode:  %s\n\n", COMMON_ANODE ? "Common-ANODE (active LOW)" : "Common-CATHODE (active HIGH)");

  /* 1. Init GPIO */
  if (gpio_init() < 0)
  {
    fprintf(stderr, "gpio_init() failed. Are you on a RPi5? Try: sudo ./test_rgb_led\n");
    return 1;
  }
  printf("[OK] gpio_init()\n");

  /* 2. Open all three pins as outputs */
  pin_t pr = pinopen(PIN_RED,   OUTPUT);
  pin_t pg = pinopen(PIN_GREEN, OUTPUT);
  pin_t pb = pinopen(PIN_BLUE,  OUTPUT);

  if (pr.mode == UNDEF || pg.mode == UNDEF || pb.mode == UNDEF)
  {
    fprintf(stderr, "Failed to open one or more RGB pins\n");
    gpio_cleanup();
    return 1;
  }
  printf("[OK] pinopen() for R/G/B\n");

  /* Start with LED off */
  rgb_off();

  /* ---- Test 1: Individual colours ---- */
  printf("\n--- Test 1: Colour sequence (%d ms each) ---\n", COLOUR_HOLD_MS);
  for (i = 0; i < NUM_COLOURS && running; i++)
  {
    printf("  %-8s  R=%d G=%d B=%d\n", colours[i].name,
           colours[i].r, colours[i].g, colours[i].b);
    rgb_set(colours[i].r, colours[i].g, colours[i].b);
    usleep(COLOUR_HOLD_MS * 1000);
  }
  rgb_off();

  /* ---- Test 2: Chase animation (R -> G -> B) ---- */
  if (running)
  {
    printf("\n--- Test 2: Chase animation (%d rounds, %d ms step) ---\n",
           CYCLE_ROUNDS, CYCLE_MS);

    for (round = 0; round < CYCLE_ROUNDS && running; round++)
    {
      printf("  Round %d: ", round + 1);

      rgb_set(1, 0, 0);
      printf("R ");
      fflush(stdout);
      usleep(CYCLE_MS * 1000);

      rgb_set(0, 1, 0);
      printf("G ");
      fflush(stdout);
      usleep(CYCLE_MS * 1000);

      rgb_set(0, 0, 1);
      printf("B ");
      fflush(stdout);
      usleep(CYCLE_MS * 1000);

      rgb_off();
      printf("\n");
    }
  }

  /* ---- Test 3: Blink all (white) ---- */
  if (running)
  {
    printf("\n--- Test 3: Blink WHITE (5 times, 300 ms) ---\n");
    for (i = 0; i < 5 && running; i++)
    {
      rgb_set(1, 1, 1);
      usleep(300 * 1000);
      rgb_off();
      usleep(300 * 1000);
      printf("  blink %d\n", i + 1);
    }
  }

  /* ---- Test 4: Toggle demo using pintoggle() ---- */
  if (running)
  {
    printf("\n--- Test 4: pintoggle() demo ---\n");
    rgb_off();
    for (i = 0; i < 6 && running; i++)
    {
      pintoggle(PIN_RED);
      pintoggle(PIN_GREEN);
      pintoggle(PIN_BLUE);
      printf("  toggle %d -> R=%d G=%d B=%d\n", i + 1,
             pinread(PIN_RED), pinread(PIN_GREEN), pinread(PIN_BLUE));
      usleep(400 * 1000);
    }
  }

  /* ---- Cleanup ---- */
  printf("\nCleaning up...\n");
  rgb_off();
  pinclose(PIN_RED);
  pinclose(PIN_GREEN);
  pinclose(PIN_BLUE);
  gpio_cleanup();
  printf("[OK] Done.\n");

  return 0;
}
