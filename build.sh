sudo g++ -c gpio_rpi5.c -o gpio_rpi5.o
sudo ar rcs gpio_rpi5.a gpio_rpi5.o
sudo g++ main.cpp -L. -l:gpio_rpi5.a -o myprogram
sudo ./myprogram 