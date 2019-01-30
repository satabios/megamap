#include <wiringPiI2C.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <wiringPi.h>
#define pi 3.14159265

int setup_magneto(void);
double get_magneto(int);