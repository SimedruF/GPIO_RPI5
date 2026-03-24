@echo off
REM Build and test GPIO RPI5 library on Windows (pinctrl backend only)
REM Direct register access requires Linux/RPi5. This builds the pinctrl version.
REM Requires GCC (MinGW) in PATH

echo === Compiling gpio_rpi5 library (pinctrl backend) ===
gcc -c gpio_rpi5_pinctrl.c -o gpio_rpi5.o -Wall -Wextra
if %ERRORLEVEL% neq 0 (
    echo ERROR: Failed to compile gpio_rpi5_pinctrl.c
    exit /b 1
)

echo === Creating static library ===
ar rcs gpio_rpi5.a gpio_rpi5.o
if %ERRORLEVEL% neq 0 (
    echo ERROR: Failed to create static library
    exit /b 1
)

echo === Compiling test program ===
g++ test\main.cpp -I. -L. -l:gpio_rpi5.a -o test\myprogram.exe -Wall -Wextra
if %ERRORLEVEL% neq 0 (
    echo ERROR: Failed to compile test program
    exit /b 1
)

echo === Build successful ===
echo Run test\myprogram.exe to execute
