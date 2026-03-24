#!/bin/bash
# Build PCM/I2S test
set -e
cd "$(dirname "$0")/.."

BACKEND="${1:-gpio_rpi5.c}"
[ "$1" = "pinctrl" ] && BACKEND="gpio_rpi5_pinctrl.c"

gcc -c "$BACKEND" -o gpio_rpi5.o -Wall -Wextra
ar rcs gpio_rpi5.a gpio_rpi5.o
g++ test/test_pcm.cpp -I. -L. -l:gpio_rpi5.a -o test/test_pcm -Wall -Wextra -lm

echo "Build OK: test/test_pcm"
echo "Run: sudo ./test/test_pcm"
