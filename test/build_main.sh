#!/bin/bash
# Build main test
set -e
cd "$(dirname "$0")/.."

BACKEND="${1:-gpio_rpi5.c}"
[ "$1" = "pinctrl" ] && BACKEND="gpio_rpi5_pinctrl.c"

gcc -c "$BACKEND" -o gpio_rpi5.o -Wall -Wextra
ar rcs gpio_rpi5.a gpio_rpi5.o
g++ test/main.cpp -I. -L. -l:gpio_rpi5.a -o test/myprogram -Wall -Wextra

echo "Build OK: test/myprogram"
echo "Run: sudo ./test/myprogram"
