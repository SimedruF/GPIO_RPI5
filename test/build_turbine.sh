#!/bin/bash
# Build Turbine Monitoring Framework
set -e
cd "$(dirname "$0")/.."

BACKEND="${1:-gpio_rpi5.c}"
[ "$1" = "pinctrl" ] && BACKEND="gpio_rpi5_pinctrl.c"

gcc -c "$BACKEND" -o gpio_rpi5.o -Wall -Wextra
ar rcs gpio_rpi5.a gpio_rpi5.o
g++ test/test_turbine.cpp -I. -L. -l:gpio_rpi5.a -o test/test_turbine -Wall -Wextra -O2 -lm

echo "Build OK: test/test_turbine"
echo "Run:"
echo "  sudo ./test/test_turbine              # Monitor 30 seconds"
echo "  sudo ./test/test_turbine 60           # Monitor 60 seconds"
echo "  sudo ./test/test_turbine log          # Monitor + CSV logging"
echo "  sudo ./test/test_turbine sim          # Simulation (no hardware)"
echo "  sudo ./test/test_turbine sar          # Test with SAR ADC backend"
