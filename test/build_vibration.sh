#!/bin/bash
# Build Vibration Monitoring test
set -e
cd "$(dirname "$0")/.."

BACKEND="${1:-gpio_rpi5.c}"
[ "$1" = "pinctrl" ] && BACKEND="gpio_rpi5_pinctrl.c"

gcc -c "$BACKEND" -o gpio_rpi5.o -Wall -Wextra
ar rcs gpio_rpi5.a gpio_rpi5.o
g++ test/test_vibration.cpp -I. -L. -l:gpio_rpi5.a -o test/test_vibration -Wall -Wextra -O2 -lm

echo "Build OK: test/test_vibration"
echo "Run:"
echo "  sudo ./test/test_vibration              # Capture 1s, print stats"
echo "  sudo ./test/test_vibration rate          # Benchmark capture rate"
echo "  sudo ./test/test_vibration fft           # Spectral analysis"
echo "  sudo ./test/test_vibration log           # Capture + CSV"
echo "  sudo ./test/test_vibration dual          # AIN0 + AIN1 interleaved"
echo "  sudo ./test/test_vibration all           # All tests"
