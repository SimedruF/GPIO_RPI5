#!/bin/bash
# Build Blade Detection test
set -e
cd "$(dirname "$0")/.."

BACKEND="${1:-gpio_rpi5.c}"
[ "$1" = "pinctrl" ] && BACKEND="gpio_rpi5_pinctrl.c"

gcc -c "$BACKEND" -o gpio_rpi5.o -Wall -Wextra
ar rcs gpio_rpi5.a gpio_rpi5.o
g++ test/test_blade_detect.cpp -I. -L. -l:gpio_rpi5.a -o test/test_blade_detect -Wall -Wextra -O2

echo "Build OK: test/test_blade_detect"
echo "Run:"
echo "  sudo ./test/test_blade_detect              # RPM measurement (10s)"
echo "  sudo ./test/test_blade_detect stress        # GPIO polling benchmark"
echo "  sudo ./test/test_blade_detect log            # RPM + CSV logging"
echo "  sudo ./test/test_blade_detect analyze        # Blade timing jitter"
echo "  sudo ./test/test_blade_detect all            # All tests"
