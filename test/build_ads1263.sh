#!/bin/bash
# Build ADS1263 High-Precision ADC HAT test
set -e
cd "$(dirname "$0")/.."

BACKEND="${1:-gpio_rpi5.c}"
[ "$1" = "pinctrl" ] && BACKEND="gpio_rpi5_pinctrl.c"

gcc -c "$BACKEND" -o gpio_rpi5.o -Wall -Wextra
ar rcs gpio_rpi5.a gpio_rpi5.o
g++ test/test_ads1263.cpp -I. -L. -l:gpio_rpi5.a -o test/test_ads1263 -Wall -Wextra

echo "Build OK: test/test_ads1263"
echo "Run:"
echo "  sudo ./test/test_ads1263           # read all 10 channels"
echo "  sudo ./test/test_ads1263 loop      # continuous reading"
echo "  sudo ./test/test_ads1263 adc2      # include ADC2 (24-bit)"
echo "  sudo ./test/test_ads1263 cal       # calibrate first"
echo "  sudo ./test/test_ads1263 all       # all tests"
