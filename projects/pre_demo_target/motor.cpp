
#include "motor.h"

int dut1=30;
int dut2=25;
int dut3=25;
int dut4=25;

int dut_let_right=25;	//40
int dut_back=25;
/*
void motor1_start(){
	softPwmCreate (pwm1_pin,0,100);
	//printf("motor1 I'm heree");
	pinMode(dir1_pin,OUTPUT);
	digitalWrite(dir1_pin,LOW);
	
	pinMode(brk1_pin,OUTPUT);
	digitalWrite(brk1_pin,HIGH);
}

void motor2_start(){
	
	pinMode(dir2_pin,OUTPUT);
	digitalWrite(dir2_pin,LOW);
	
	pinMode(brk2_pin,OUTPUT);
	digitalWrite(brk2_pin,LOW);
	
	softPwmCreate (pwm2_pin,dut2,100);
}


void motor3_start(){
	softPwmCreate (pwm3_pin,0,100);
	
	pinMode(dir3_pin,OUTPUT);
	digitalWrite(dir3_pin,HIGH);
	
	pinMode(brk3_pin,OUTPUT);
	digitalWrite(brk3_pin,HIGH);
}

void motor4_start(){
	
	pinMode(dir4_pin,OUTPUT);
	digitalWrite(dir4_pin,HIGH);
	
	pinMode(brk4_pin,OUTPUT);
	digitalWrite(brk4_pin,LOW);
	
	softPwmCreate (pwm4_pin,dut2,100);
}
*/


void motor2_start(){
	softPwmCreate (pwm2_pin,0,100);
	//printf("motor1 I'm heree");
	pinMode(dir2_pin,OUTPUT);
	digitalWrite(dir2_pin,LOW);
	
	pinMode(brk2_pin,OUTPUT);
	digitalWrite(brk2_pin,HIGH);
}

void motor1_start(){
	
	pinMode(dir1_pin,OUTPUT);
	digitalWrite(dir1_pin,HIGH);
	
	pinMode(brk1_pin,OUTPUT);
	digitalWrite(brk1_pin,LOW);
	
	softPwmCreate (pwm1_pin,dut1,100);
}


void motor4_start(){
	softPwmCreate (pwm4_pin,0,100);
	
	pinMode(dir4_pin,OUTPUT);
	digitalWrite(dir4_pin,HIGH);
	
	pinMode(brk4_pin,OUTPUT);
	digitalWrite(brk4_pin,HIGH);
}

void motor3_start(){
	
	pinMode(dir3_pin,OUTPUT);
	digitalWrite(dir3_pin,LOW);
	
	pinMode(brk3_pin,OUTPUT);
	digitalWrite(brk3_pin,LOW);
	
	softPwmCreate (pwm3_pin,dut3,100);
}

void stop_robot(){
	digitalWrite(brk1_pin,HIGH);
	digitalWrite(brk2_pin,HIGH);
	digitalWrite(brk3_pin,HIGH);
	digitalWrite(brk4_pin,HIGH);
}

void start_straight(void)
{
  motor1_start();
  motor2_start();
  motor3_start();
  motor4_start();
  softPwmWrite(pwm1_pin,30);
  softPwmWrite(pwm3_pin,25);
}


void take_left(int delay1){
	digitalWrite(brk1_pin,LOW);
	digitalWrite(brk2_pin,LOW);
	digitalWrite(brk3_pin,LOW);
	digitalWrite(brk4_pin,LOW);
	
	digitalWrite(dir2_pin,LOW);
	digitalWrite(dir4_pin,LOW);
	digitalWrite(dir1_pin,LOW);
	digitalWrite(dir3_pin,LOW);

	softPwmWrite(pwm2_pin,dut_let_right);
	softPwmWrite(pwm4_pin,dut_let_right);
	softPwmWrite(pwm1_pin,dut_let_right);
	softPwmWrite(pwm3_pin,dut_let_right);
	
	delay(delay1);
	
	digitalWrite(brk1_pin,HIGH);
	digitalWrite(brk2_pin,HIGH);
	digitalWrite(brk3_pin,HIGH);
	digitalWrite(brk4_pin,HIGH);
}

void take_right(int delay1){
	digitalWrite(brk1_pin,LOW);
	digitalWrite(brk2_pin,LOW);
	digitalWrite(brk3_pin,LOW);
	digitalWrite(brk4_pin,LOW);
	
	digitalWrite(dir2_pin,HIGH);
	digitalWrite(dir4_pin,HIGH);
	digitalWrite(dir1_pin,HIGH);
	digitalWrite(dir3_pin,HIGH);

	softPwmWrite(pwm2_pin,dut_let_right);
	softPwmWrite(pwm4_pin,dut_let_right);
	softPwmWrite(pwm1_pin,dut_let_right);
	softPwmWrite(pwm1_pin,dut_let_right);
	
	delay(delay1);
	
	digitalWrite(brk1_pin,HIGH);
	digitalWrite(brk2_pin,HIGH);
	digitalWrite(brk3_pin,HIGH);
	digitalWrite(brk4_pin,HIGH);
}

void continue_straight(){
 digitalWrite(brk1_pin,LOW);
 digitalWrite(brk3_pin,LOW);
 digitalWrite(brk2_pin,HIGH);
 digitalWrite(brk4_pin,HIGH); 

  softPwmWrite(pwm1_pin,45);
  softPwmWrite(pwm2_pin,0);
  softPwmWrite(pwm3_pin,30);
  softPwmWrite(pwm4_pin,0);
  
  delay(125);
  
  softPwmWrite(pwm1_pin,dut1);
  softPwmWrite(pwm2_pin,0);
  softPwmWrite(pwm3_pin,dut3);
  softPwmWrite(pwm4_pin,0);
  
  digitalWrite(dir1_pin,HIGH);
  digitalWrite(dir3_pin,LOW);
  
  digitalWrite(brk1_pin,LOW);
  digitalWrite(brk2_pin,HIGH);
  digitalWrite(brk3_pin,LOW);
  digitalWrite(brk4_pin,HIGH);
}

void move_back(int delay1){
  
  softPwmWrite(pwm1_pin,dut_back);
  softPwmWrite(pwm2_pin,0);
  softPwmWrite(pwm3_pin,dut_back);
  softPwmWrite(pwm4_pin,0);
  
  digitalWrite(dir1_pin,LOW);
  digitalWrite(dir3_pin,HIGH);
  
  digitalWrite(brk1_pin,LOW);
  digitalWrite(brk2_pin,HIGH);
  digitalWrite(brk3_pin,LOW);
  digitalWrite(brk4_pin,HIGH);
  
  delay(delay1);
  
  digitalWrite(brk1_pin,HIGH);
  digitalWrite(brk2_pin,HIGH);
  digitalWrite(brk3_pin,HIGH);
  digitalWrite(brk4_pin,HIGH);
}
