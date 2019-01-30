
#include "motor.h"

int dut1=17;
int dut2=17;

int dut_let_right=40;	//50
int dut_back=25;

void motor1_start(){
	softPwmCreate (pwm1_pin,dut1,100);
	//printf("motor1 I'm heree");
	pinMode(dir1_pin,OUTPUT);
	digitalWrite(dir1_pin,LOW);
	
	pinMode(brk1_pin,OUTPUT);
	digitalWrite(brk1_pin,LOW);
}

void motor2_start(){
	softPwmCreate (pwm2_pin,dut2,100);
	//printf("motor1 I'm heree");
	
	pinMode(dir2_pin,OUTPUT);
	digitalWrite(dir2_pin,HIGH);
	
	pinMode(brk2_pin,OUTPUT);
	digitalWrite(brk2_pin,LOW);
}

void stop_robot(){
	digitalWrite(brk1_pin,HIGH);
	digitalWrite(brk2_pin,HIGH);
}

void take_left(int delay1){
	digitalWrite(brk1_pin,LOW);
	digitalWrite(brk2_pin,LOW);
	
	softPwmWrite(pwm1_pin,dut_let_right);
	softPwmWrite(pwm2_pin,dut_let_right);
		
	digitalWrite(dir2_pin,HIGH);
	digitalWrite(dir1_pin,HIGH);
	
	delay(delay1);
	
	digitalWrite(brk1_pin,HIGH);
	digitalWrite(brk2_pin,HIGH);
}

void take_right(int delay1){
	digitalWrite(brk1_pin,LOW);
	digitalWrite(brk2_pin,LOW);
	
	softPwmWrite(pwm1_pin,dut_let_right);
	softPwmWrite(pwm2_pin,dut_let_right);
	
	digitalWrite(dir2_pin,LOW);
	digitalWrite(dir1_pin,LOW);

	
	delay(delay1);
	
	digitalWrite(brk1_pin,HIGH);
	digitalWrite(brk2_pin,HIGH);
}

void continue_straight(){
  
  softPwmWrite(pwm1_pin,dut1);
  softPwmWrite(pwm2_pin,dut2);
  
  pinMode(dir1_pin,OUTPUT);
  digitalWrite(dir1_pin,LOW);
  
  pinMode(brk1_pin,OUTPUT);
  digitalWrite(brk1_pin,LOW);

  pinMode(dir2_pin,OUTPUT);
  digitalWrite(dir2_pin,HIGH);
  
  pinMode(brk2_pin,OUTPUT);
  digitalWrite(brk2_pin,LOW);
}

void move_back(int delay1){
  pinMode(brk1_pin,OUTPUT);
  pinMode(brk2_pin,OUTPUT);
  digitalWrite(brk1_pin,LOW);
  digitalWrite(brk2_pin,LOW);
	
  softPwmWrite(pwm1_pin,dut_back);
  softPwmWrite(pwm2_pin,dut_back);
  
  pinMode(dir1_pin,OUTPUT);
  digitalWrite(dir1_pin,HIGH);
  
  pinMode(dir2_pin,OUTPUT);
  digitalWrite(dir2_pin,LOW);
  
  delay(delay1);
  
  digitalWrite(brk1_pin,HIGH);
  digitalWrite(brk2_pin,HIGH);  
}
