
#include "motor.h"

int dut1=15;


int dut2=15;
volatile int eventCounter2 = 0;

void motor1_start(){
	softPwmCreate (pwm1_pin,dut1,100);
	
	pinMode(dir1_pin,OUTPUT);
	digitalWrite(dir1_pin,HIGH);
	
	pinMode(brk1_pin,OUTPUT);
	digitalWrite(brk1_pin,LOW);
}

void motor2_start(){
	softPwmCreate (pwm2_pin,dut2,100);
	
	pinMode(dir2_pin,OUTPUT);
	digitalWrite(dir2_pin,LOW);
	
	pinMode(brk2_pin,OUTPUT);
	digitalWrite(brk2_pin,LOW);
}



void myInterrupt2(void) {
   eventCounter2++;
}