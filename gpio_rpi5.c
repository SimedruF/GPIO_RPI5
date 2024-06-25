// Compile with $ gcc gpio_rpi5.c -o gpio_rpi5

#include <sys/stat.h>
#include <sys/types.h>

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cstring>
#include "gpio_rpi5.h"

// Pin modes:
#define INPUT (0)
#define OUTPUT (1)
#define LOW (0)
#define HIGH (1)
#define UNDEF (3)
#define TEST_PIN 17
pin_t rpi5_gpio[237];

int gpio_pintest(int pin_indx)
{
  int time = 0;
  pin_t testpin = pinopen(pin_indx, OUTPUT);
  pinwrite(pin_indx, HIGH);
  sleep(1000);
  pinwrite(pin_indx, LOW);
  pinclose(pin_indx);
  return 0;
}

pin_t pinopen(int pin, int mode)
{
   char *cmd_openPin = (char*)malloc(1024);
   char*   pinfn = (char*)malloc(1024);
   FILE *pf;
   char data[512];
   if(mode == OUTPUT)
   {
     snprintf(cmd_openPin, 1024, "pinctrl set %d op", pin);
   }
   else
   {
      snprintf(cmd_openPin, 1024, "pinctrl set %d ip", pin);
   }

   pf = popen(cmd_openPin,"r"); 
   if(!pf)
   {
     fprintf(stderr, "pinopen: Could not open pipe for output.\n");
     exit(1);
   }

   if (pclose(pf) != 0)
       fprintf(stderr," pinopen Error: Failed to close command stream \n");
   
   rpi5_gpio[pin].mode =  mode;
   return (pin_t) { UNDEF, mode };
}

void pinclose(int indx_pin)
{
  rpi5_gpio[indx_pin].mode =  UNDEF;
  rpi5_gpio[indx_pin].state =  UNDEF;
}

void pinwrite(int indx_pin, int value)
{
   char *cmd_openPin = (char*)malloc(1024);
   char*   pinfn = (char*)malloc(1024);
   FILE *pf;
   char data[512];
   if(value == LOW)
   {
     snprintf(cmd_openPin, 1024, "pinctrl set %d dl", indx_pin);
   }
   else
   {
      snprintf(cmd_openPin, 1024, "pinctrl set %d dh", indx_pin);
   }
   pf = popen(cmd_openPin,"r"); 
   if(!pf)
   {
     fprintf(stderr, "pinwrite : Could not open pipe for output.\n");
     exit(1);
   }

   if (pclose(pf) != 0)
   {
     fprintf(stderr," pinwrite Error: Failed to close command stream \n");
     exit(1);
   } 
   rpi5_gpio[indx_pin].state =  value;
}

int pinread(int pin)
{
  char *cmd_openPin = (char*)malloc(1024);
  FILE *pf;
  char data[512];
  int pinout = 0;
  char *pin_off = (char*)"lo";
  char *pin_on = (char*)"hi";
  snprintf(cmd_openPin, 1024, "pinctrl get %d ", pin);

  pf = popen(cmd_openPin,"r"); 
  if(!pf)
  {
    fprintf(stderr, "Could not open pipe for output.\n");
    exit(1);
  }
 
  // Grab data from process execution
  fgets(data, 512 , pf);
  // Print grabbed data to the screen.
  fprintf(stdout, "-%s-\n",data); 
 
  char *pch_off = strstr(data, pin_off);
  char *pch_on = strstr(data, pin_on);

  if(pch_off)
  {
    fprintf(stdout, "-%s-\n",pch_off); 
    pinout = 0;
  }
  if(pch_on)
  {
    fprintf(stdout, "-%s-\n",pch_on); 
    pinout = 1;
  }
  rpi5_gpio[pin].state =  pinout;

  if (pclose(pf) != 0)
  {
   fprintf(stderr," pinread Error: Failed to close command stream \n");
   exit(1);
  }

  return pinout;

}