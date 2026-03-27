#!/bin/bash
#
# Build script for AD7606 test
#
# Usage: ./build_ad7606.sh
# Run:   sudo ./test_ad7606 [rate|oversample|stats|capture|cont|verify|all]
#

set -e

BACKEND="../gpio_rpi5.c"
PINCTRL="../gpio_rpi5_pinctrl.c"

echo "=== Building AD7606 Test ==="

# Compile GPIO backend
echo "[1/3] Compiling GPIO backend..."
gcc -c "$BACKEND" -o gpio_rpi5.o -Wall -Wextra -O2
gcc -c "$PINCTRL" -o gpio_rpi5_pinctrl.o -Wall -Wextra -O2

# Create static library
echo "[2/3] Creating library..."
ar rcs gpio_rpi5.a gpio_rpi5.o gpio_rpi5_pinctrl.o

# Build test
echo "[3/3] Building test_ad7606..."
g++ test_ad7606.cpp -I.. -L. -l:gpio_rpi5.a -o test_ad7606 -Wall -Wextra -O2 -lm

echo ""
echo "=== Build OK ==="
echo "Run:  sudo ./test_ad7606          # read 8 channels once"
echo "      sudo ./test_ad7606 rate     # benchmark speed"
echo "      sudo ./test_ad7606 all      # run all tests"
echo "      sudo ./test_ad7606 help     # show all options"
