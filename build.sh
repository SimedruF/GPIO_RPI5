#!/bin/bash
# Build GPIO RPI5 library - Direct register access (default)
# Use: ./build.sh          -> direct register access (fast, needs /dev/gpiomem0)
# Use: ./build.sh pinctrl  -> pinctrl backend (slower, works without root for gpiomem)

BACKEND="gpio_rpi5.c"
if [ "$1" = "pinctrl" ]; then
  BACKEND="gpio_rpi5_pinctrl.c"
  echo "=== Building with pinctrl backend ==="
else
  echo "=== Building with direct register access ==="
fi

set -e

echo "=== Compiling $BACKEND ==="
sudo gcc -c "$BACKEND" -o gpio_rpi5.o -Wall -Wextra

echo "=== Creating static library ==="
sudo ar rcs gpio_rpi5.a gpio_rpi5.o

echo "=== Compiling test program ==="
sudo g++ test/main.cpp -I. -L. -l:gpio_rpi5.a -o test/myprogram

echo "=== Build successful ==="
echo "Run: sudo ./test/myprogram"