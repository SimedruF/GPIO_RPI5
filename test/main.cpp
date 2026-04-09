#include "../gpio_rpi5.h"
#include <stdio.h>

int main()
{
  if (gpio_init() < 0)
  {
    fprintf(stderr, "Failed to initialize GPIO. Are you on a Raspberry Pi 5?\n");
    return 1;
  }

  printf("GPIO initialized successfully.\n");

  /* Test: blink GPIO4 */
  printf("Testing GPIO%d: HIGH for 1s, then LOW\n", GPIO4);
  (void)gpio_pintest(GPIO4);

  /* Test: toggle GPIO17 */
  printf("Testing pintoggle on GPIO%d\n", GPIO17);
  pinopen(GPIO17, OUTPUT);
  pinwrite(GPIO17, LOW);
  pintoggle(GPIO17);
  printf("GPIO%d state after toggle: %d\n", GPIO17, pinread(GPIO17));
  pintoggle(GPIO17);
  printf("GPIO%d state after 2nd toggle: %d\n", GPIO17, pinread(GPIO17));
  pinclose(GPIO17);

  gpio_cleanup();
  printf("GPIO cleanup done.\n");
  return 0;
}