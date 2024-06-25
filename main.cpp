#include "gpio_rpi5.h"

int main()
{
  (void)gpio_pintest(GPIO4);
  return 1;
}