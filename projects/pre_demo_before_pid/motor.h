#include <wiringPi.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <softPwm.h>

////////////////////////////////////////////////////////////
#define pwm1_pin 0
#define brk1_pin 2
#define dir1_pin 3

void motor1_start(void);
///////////////////////////////////////////////////////////
#define pwm2_pin 15
#define brk2_pin 16
#define dir2_pin 1

void motor2_start(void);
///////////////////////////////////////////////////////////

#define pwm3_pin 13
#define brk3_pin 14
#define dir3_pin 12

void motor3_start(void);
///////////////////////////////////////////////////////////

#define pwm4_pin 10
#define brk4_pin 11
#define dir4_pin 6

void motor4_start(void);
//////////////////////////////////////////////////////////

void start_straight(void);
void move_forward();
void stop_robot();
void take_left(int delay1);
void take_right(int delay1);
void continue_straight(void);
void move_back(int delay1);
///////////////////////////////////////////////////////////