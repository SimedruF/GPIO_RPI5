This small and simple library for C it is using the pinctrl application on Raspberry Pi OS.The pinctrl subsystem in Linux is used to configure the behavior of pins on the GPIO (General Purpose Input/Output) interface. 
Raspberry Pi OS uses the pinctrl subsystem to manage pin configuration and settings.

Command line usage of pinctrl: 

$ pinctrl set [pin] [mode] [state]

Compile with gcc:

$ g++ -c gpio_rpi5.c -o gpio_rpi5.o

$ ar rcs gpio_rpi5.a gpio_rpi5.o

$ gcc main.c -L. -lgpio_rpi5 -o myprogram
