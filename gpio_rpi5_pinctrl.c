/**
 * @file gpio_rpi5_pinctrl.c
 * @brief Pinctrl CLI fallback GPIO backend for Raspberry Pi 5.
 *
 * This backend executes the `pinctrl` command-line tool (via `popen()`) to
 * control GPIO pins. It is significantly slower than the mmap backend (~10 ms
 * per operation due to process spawning) but supports all GPIO ranges:
 * - RP1 GPIOs 0–53 (40-pin header)
 * - BCM2712 GPIOs 100–135 (processor-side)
 * - AON GPIOs 200–237 (always-on domain)
 *
 * @par When to Use This Backend
 * - When you need to access GPIOs outside the RP1 range (100–237).
 * - When `/dev/gpiomem0` is not available.
 * - For quick prototyping where speed is not critical.
 *
 * @par Limitations
 * - pin_set_drive() is not supported (returns -1).
 * - pintoggle() is implemented as pinread() + pinwrite() (two CLI calls).
 *
 * Compile with: gcc -c gpio_rpi5_pinctrl.c -o gpio_rpi5_pinctrl.o -Wall -Wextra
 */

#include <sys/stat.h>
#include <sys/types.h>

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "gpio_rpi5.h"


/** @brief Global array caching the state of all GPIO pins. */
pin_t rpi5_gpio[GPIO_MAX_INDEX + 1];

/** @brief Initialization flag (1 after successful gpio_init()). */
static int gpio_initialized = 0;

/**
 * @brief Validate that a pin index is within any known GPIO range.
 *
 * Unlike the mmap backend (which only supports 0–53), this backend accepts:
 * - 0–53 (RP1 GPIOs)
 * - 100–135 (BCM2712 GPIOs)
 * - 200–237 (AON GPIOs)
 *
 * @param pin GPIO number to validate.
 * @return 1 if valid, 0 otherwise.
 */
static int gpio_valid_pin(int pin)
{
  if (pin >= 0 && pin <= 53)   return 1;  /* RP1 GPIOs */
  if (pin >= 100 && pin <= 135) return 1;  /* BCM2712 GPIOs */
  if (pin >= 200 && pin <= 237) return 1;  /* AON GPIOs */
  return 0;
}

/**
 * @brief Initialize the GPIO subsystem (pinctrl backend).
 *
 * Resets all pin states to UNDEF and sets the initialization flag.
 * Unlike the mmap backend, no device files are opened — each
 * subsequent operation spawns a `pinctrl` CLI process.
 *
 * @return Always returns 0 (cannot fail).
 */
int gpio_init(void)
{
  int i;
  for (i = 0; i <= GPIO_MAX_INDEX; i++) {
    rpi5_gpio[i].mode  = UNDEF;
    rpi5_gpio[i].state = UNDEF;
  }
  gpio_initialized = 1;
  return 0;
}

/**
 * @brief Release GPIO resources (pinctrl backend).
 *
 * Clears the initialization flag. No file descriptors or memory
 * mappings need to be released in this backend.
 */
void gpio_cleanup(void)
{
  gpio_initialized = 0;
}

/**
 * @brief Quick pin test: pulse a pin HIGH for 1 second (pinctrl backend).
 *
 * Opens the pin as OUTPUT, drives HIGH, waits 1 second, drives LOW,
 * then closes. Executes 4 `pinctrl` commands total.
 *
 * @param pin_indx GPIO number to test (0–53, 100–135, or 200–237).
 * @return 0 on success, -1 if the pin is invalid or a command fails.
 */
int gpio_pintest(int pin_indx)
{
  if (!gpio_valid_pin(pin_indx)) return -1;

  pin_t testpin = pinopen(pin_indx, OUTPUT);
  if (testpin.mode == UNDEF && testpin.state == UNDEF)
    return -1;

  pinwrite(pin_indx, HIGH);
  sleep(1);
  pinwrite(pin_indx, LOW);
  pinclose(pin_indx);
  return 0;
}

/**
 * @brief Open a GPIO pin in the specified direction (pinctrl backend).
 *
 * Runs `pinctrl set <pin> op` (output) or `pinctrl set <pin> ip` (input)
 * via popen(). The command spawns a separate process, making this ~1000x
 * slower than the mmap backend.
 *
 * @param pin  GPIO number (0–53, 100–135, or 200–237).
 * @param mode Direction: INPUT (0) or OUTPUT (1).
 * @return pin_t with the configured mode. On error, both fields are UNDEF.
 */
pin_t pinopen(int pin, int mode)
{
   char cmd[128];
   FILE *pf;
   pin_t err_pin = { UNDEF, UNDEF };

   if (!gpio_valid_pin(pin))
   {
     fprintf(stderr, "pinopen: Invalid pin %d\n", pin);
     return err_pin;
   }

   if (mode == OUTPUT)
     snprintf(cmd, sizeof(cmd), "pinctrl set %d op", pin);
   else
     snprintf(cmd, sizeof(cmd), "pinctrl set %d ip", pin);

   pf = popen(cmd, "r");
   if (!pf)
   {
     fprintf(stderr, "pinopen: Could not open pipe for pin %d\n", pin);
     return err_pin;
   }

   if (pclose(pf) != 0)
   {
     fprintf(stderr, "pinopen: Failed to close command stream for pin %d\n", pin);
     return err_pin;
   }

   rpi5_gpio[pin].mode = mode;
   return (pin_t) { UNDEF, mode };
}

/**
 * @brief Close a GPIO pin and reset to input (pinctrl backend).
 *
 * Runs `pinctrl set <pin> ip` to reset the pin to a safe input state.
 * The cached mode and state are reset to UNDEF.
 *
 * @param indx_pin GPIO number to close.
 */
void pinclose(int indx_pin)
{
  char cmd[128];
  FILE *pf;

  if (!gpio_valid_pin(indx_pin)) return;

  /* Reset pin to input (safe default) */
  snprintf(cmd, sizeof(cmd), "pinctrl set %d ip", indx_pin);
  pf = popen(cmd, "r");
  if (pf) pclose(pf);

  rpi5_gpio[indx_pin].mode  = UNDEF;
  rpi5_gpio[indx_pin].state = UNDEF;
}

/**
 * @brief Write a digital value to a GPIO pin (pinctrl backend).
 *
 * Runs `pinctrl set <pin> dl` (drive low) or `pinctrl set <pin> dh`
 * (drive high). Implicitly sets the pin as output.
 *
 * @param indx_pin GPIO number (must be a valid pin).
 * @param value    Desired level: HIGH (1) or LOW (0).
 * @return 0 on success, -1 on error (invalid pin or command failure).
 */
int pinwrite(int indx_pin, int value)
{
   char cmd[128];
   FILE *pf;

   if (!gpio_valid_pin(indx_pin))
   {
     fprintf(stderr, "pinwrite: Invalid pin %d\n", indx_pin);
     return -1;
   }

   if (value == LOW)
     snprintf(cmd, sizeof(cmd), "pinctrl set %d dl", indx_pin);
   else
     snprintf(cmd, sizeof(cmd), "pinctrl set %d dh", indx_pin);

   pf = popen(cmd, "r");
   if (!pf)
   {
     fprintf(stderr, "pinwrite: Could not open pipe for pin %d\n", indx_pin);
     return -1;
   }

   if (pclose(pf) != 0)
   {
     fprintf(stderr, "pinwrite: Failed to close command stream for pin %d\n", indx_pin);
     return -1;
   }

   rpi5_gpio[indx_pin].state = value;
   return 0;
}

/**
 * @brief Read the current logic level of a GPIO pin (pinctrl backend).
 *
 * Runs `pinctrl get <pin>` and parses the output for "| lo" or "| hi"
 * to determine the logic level. The pipe delimiter avoids false matches
 * with pin name substrings.
 *
 * @param pin GPIO number.
 * @return HIGH (1), LOW (0), or -1 on error or if the level cannot be parsed.
 */
int pinread(int pin)
{
  char cmd[128];
  FILE *pf;
  char data[512];
  int pinout = -1;

  if (!gpio_valid_pin(pin))
  {
    fprintf(stderr, "pinread: Invalid pin %d\n", pin);
    return -1;
  }

  snprintf(cmd, sizeof(cmd), "pinctrl get %d", pin);

  pf = popen(cmd, "r");
  if (!pf)
  {
    fprintf(stderr, "pinread: Could not open pipe for pin %d\n", pin);
    return -1;
  }

  if (fgets(data, sizeof(data), pf) != NULL)
  {
    /* Search for "| lo" or "| hi" to avoid matching pin name substrings */
    if (strstr(data, "| lo"))
      pinout = 0;
    else if (strstr(data, "| hi"))
      pinout = 1;
  }

  if (pclose(pf) != 0)
  {
    fprintf(stderr, "pinread: Failed to close command stream for pin %d\n", pin);
    return -1;
  }

  if (pinout >= 0)
    rpi5_gpio[pin].state = pinout;

  return pinout;
}

/**
 * @brief Configure the internal pull resistor (pinctrl backend).
 *
 * Runs `pinctrl set <pin> pu|pd|pn` to set pull-up, pull-down,
 * or no-pull respectively.
 *
 * @param pin  GPIO number.
 * @param pull Pull mode: PULL_UP (1), PULL_DOWN (2), or PULL_NONE (0).
 * @return 0 on success, -1 on error.
 */
int pinpull(int pin, int pull)
{
  char cmd[128];
  FILE *pf;
  const char *pull_str;

  if (!gpio_valid_pin(pin))
  {
    fprintf(stderr, "pinpull: Invalid pin %d\n", pin);
    return -1;
  }

  switch (pull)
  {
    case PULL_UP:   pull_str = "pu"; break;
    case PULL_DOWN: pull_str = "pd"; break;
    case PULL_NONE: pull_str = "pn"; break;
    default:
      fprintf(stderr, "pinpull: Invalid pull mode %d\n", pull);
      return -1;
  }

  snprintf(cmd, sizeof(cmd), "pinctrl set %d %s", pin, pull_str);

  pf = popen(cmd, "r");
  if (!pf)
  {
    fprintf(stderr, "pinpull: Could not open pipe for pin %d\n", pin);
    return -1;
  }

  if (pclose(pf) != 0)
  {
    fprintf(stderr, "pinpull: Failed to close command stream for pin %d\n", pin);
    return -1;
  }

  return 0;
}

/**
 * @brief Toggle the output state of a GPIO pin (pinctrl backend).
 *
 * Reads the current level via pinread(), then writes the inverted value
 * via pinwrite(). Requires two `pinctrl` commands (slower than mmap XOR).
 *
 * @param pin GPIO number (should have been opened as OUTPUT).
 * @return 0 on success, -1 on error.
 */
int pintoggle(int pin)
{
  int val = pinread(pin);
  if (val < 0) return -1;
  return pinwrite(pin, val ? LOW : HIGH);
}

/**
 * @brief Set the alternate function for a GPIO pin (pinctrl backend).
 *
 * Runs `pinctrl set <pin> a<func>` to switch the pin to a hardware
 * peripheral function (SPI, I2C, UART, PWM, etc.).
 *
 * @param pin  GPIO number.
 * @param func Alternate function number: 0–8 (ALT0–ALT8) or 31 (NULL/disconnect).
 * @return 0 on success, -1 on error.
 *
 * @par Example
 * @code
 * pinalt(GPIO14, ALT4);  // Set GPIO14 to UART0 TXD
 * pinalt(GPIO15, ALT4);  // Set GPIO15 to UART0 RXD
 * @endcode
 */
int pinalt(int pin, int func)
{
  char cmd[128];
  FILE *pf;

  if (!gpio_valid_pin(pin))
  {
    fprintf(stderr, "pinalt: Invalid pin %d\n", pin);
    return -1;
  }
  if (func < 0 || (func > 8 && func != 31))
  {
    fprintf(stderr, "pinalt: Invalid function %d\n", func);
    return -1;
  }

  snprintf(cmd, sizeof(cmd), "pinctrl set %d a%d", pin, func);

  pf = popen(cmd, "r");
  if (!pf)
  {
    fprintf(stderr, "pinalt: Could not open pipe for pin %d\n", pin);
    return -1;
  }

  if (pclose(pf) != 0)
  {
    fprintf(stderr, "pinalt: Failed to close command stream for pin %d\n", pin);
    return -1;
  }

  return 0;
}

/**
 * @brief Set drive strength — NOT SUPPORTED in the pinctrl backend.
 *
 * The `pinctrl` CLI does not expose drive strength control.
 * Use the mmap backend (gpio_rpi5.c) if drive strength configuration
 * is required.
 *
 * @param pin      Ignored.
 * @param strength Ignored.
 * @return Always returns -1.
 */
int pin_set_drive(int pin, int strength)
{
  (void)pin;
  (void)strength;
  fprintf(stderr, "pin_set_drive: Not supported in pinctrl backend\n");
  return -1;
}