#!/bin/bash
# Build all test programs for GPIO_RPI5
# Usage: ./build_tests.sh [pinctrl]

set -e
cd "$(dirname "$0")/.."

BACKEND="gpio_rpi5.c"
if [ "$1" = "pinctrl" ]; then
  BACKEND="gpio_rpi5_pinctrl.c"
  echo "=== Building with pinctrl backend ==="
else
  echo "=== Building with direct register access ==="
fi

echo "=== Compiling library ($BACKEND) ==="
gcc -c "$BACKEND" -o gpio_rpi5.o -Wall -Wextra

echo "=== Creating static library ==="
ar rcs gpio_rpi5.a gpio_rpi5.o

echo "=== Compiling test programs ==="

echo "  main..."
g++ test/main.cpp -I. -L. -l:gpio_rpi5.a -o test/myprogram -Wall -Wextra

echo "  test_spi..."
g++ test/test_spi.cpp -I. -L. -l:gpio_rpi5.a -o test/test_spi -Wall -Wextra

echo "  test_pwm..."
g++ test/test_pwm.cpp -I. -L. -l:gpio_rpi5.a -o test/test_pwm -Wall -Wextra -lm

echo "  test_i2c..."
g++ test/test_i2c.cpp -I. -L. -l:gpio_rpi5.a -o test/test_i2c -Wall -Wextra

echo "  test_uart..."
g++ test/test_uart.cpp -I. -L. -l:gpio_rpi5.a -o test/test_uart -Wall -Wextra

echo "  test_pcm..."
g++ test/test_pcm.cpp -I. -L. -l:gpio_rpi5.a -o test/test_pcm -Wall -Wextra -lm

echo ""
echo "=== Build successful ==="
echo ""
echo "Run tests with sudo:"
echo "  sudo ./test/myprogram"
echo "  sudo ./test/test_spi [l]"
echo "  sudo ./test/test_pwm"
echo "  sudo ./test/test_i2c [scan]"
echo "  sudo ./test/test_uart [loop]"
echo "  sudo ./test/test_pcm"
