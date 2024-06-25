This small and simple library for C it is using the pinctrl application on Raspberry Pi OS.The pinctrl subsystem in Linux is used to configure the behavior of pins on the GPIO (General Purpose Input/Output) interface. 
Raspberry Pi OS uses the pinctrl subsystem to manage pin configuration and settings.

Command line usage of pinctrl: 

$ pinctrl set [pin] [mode] [state]

Compile with g++:

$ g++ -c gpio_rpi5.c -o gpio_rpi5.o

$ ar rcs gpio_rpi5.a gpio_rpi5.o

$ g++ main.cpp -L. -l:gpio_rpi5.a -o myprogram

or run build script 

$ chmod +x ./build.sh

$ ./build.sh

If the application pinctrl is not available the program will return "sh: 1: pinctrl: not found"