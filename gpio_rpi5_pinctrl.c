// gpio_rpi5_pinctrl.c - GPIO library using pinctrl command (fallback backend)
// Compile with: gcc -c gpio_rpi5_pinctrl.c -o gpio_rpi5_pinctrl.o -Wall -Wextra

#include <sys/stat.h>
#include <sys/types.h>

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "gpio_rpi5.h"


pin_t rpi5_gpio[GPIO_MAX_INDEX + 1];
static int gpio_initialized = 0;

/* Validate that a pin index is within known GPIO ranges */
static int gpio_valid_pin(int pin)
{
  if (pin >= 0 && pin <= 53)   return 1;  /* RP1 GPIOs */
  if (pin >= 100 && pin <= 135) return 1;  /* BCM2712 GPIOs */
  if (pin >= 200 && pin <= 237) return 1;  /* AON GPIOs */
  return 0;
}

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

void gpio_cleanup(void)
{
  gpio_initialized = 0;
}

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

int pintoggle(int pin)
{
  int val = pinread(pin);
  if (val < 0) return -1;
  return pinwrite(pin, val ? LOW : HIGH);
}

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

int pin_set_drive(int pin, int strength)
{
  (void)pin;
  (void)strength;
  fprintf(stderr, "pin_set_drive: Not supported in pinctrl backend\n");
  return -1;
}