#include <wiringPi.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <softPwm.h>

#define pwm1_pin 0
#define brk1_pin 2
#define dir1_pin 3

#define encoder1_pin 4


void motor1_start(void);
void move_forward();
///////////////////////////////////////////////////////////

#define pwm2_pin 15
#define brk2_pin 16
#define dir2_pin 1

#define encoder2_pin 5



void motor2_start(void);
void myInterrupt2(void);
///////////////////////////////////////////////////////////