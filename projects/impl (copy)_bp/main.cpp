#include "Jive.h"
//#include "motor.h"
/*
//////////////////////////////////////////////////////////////////////////////
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
void stop_robot();
void take_left(int delay1);
void take_right(int delay1);
///////////////////////////////////////////////////////////

#define pwm2_pin 15
#define brk2_pin 16
#define dir2_pin 1

#define encoder2_pin 5

void motor2_start(void);
void myInterrupt2(void);
///////////////////////////////////////////////////////////

int dut1=20;
int dut2=20;
volatile int eventCounter2 = 0;
volatile int eventCounter1 = 0;
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
	
	digitalWrite(dir2_pin,HIGH);
	digitalWrite(dir1_pin,HIGH);
	softPwmCreate(pwm1_pin,dut1,100);
	softPwmCreate(pwm2_pin,dut2,100);
	
	delay(delay1);
	
	digitalWrite(brk1_pin,HIGH);
	digitalWrite(brk2_pin,HIGH);
}

void take_right(int delay1){
	digitalWrite(brk1_pin,LOW);
	digitalWrite(brk2_pin,LOW);
	
	digitalWrite(dir2_pin,LOW);
	digitalWrite(dir1_pin,LOW);
	softPwmCreate(pwm1_pin,dut1,100);
	softPwmCreate(pwm2_pin,dut2,100);
	
	delay(delay1);
	
	digitalWrite(brk1_pin,HIGH);
	digitalWrite(brk2_pin,HIGH);
}

void myInterrupt2(void) {
   eventCounter2++;
}

void myInterrupt1(void) {
   eventCounter1++;
}

//////////////////////////////////////////////////////////////////////////////*/

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

int main(int argc, char *argv[])
{
  wiringPiSetup();
   int key;
   bool done = false;
   Mat bImg;
   /*
   cout<<"before start"<<endl;
   motor1_start();
   motor2_start();
   delay(2000);
   stop_robot();
   delay(2000);
   cout<<"before left"<<endl;
   move_back(700);
//    take_left(2000);
//    cout<<"after left"<<endl;
    delay(2000);
       stop_robot();
//    take_right(2000);
    while(1);
// */  
   
   
   /*
   double angle;
   int fd1;
   fd1=setup_magneto();
   while(1){
     
    angle=get_magneto(fd1);
    printf("%f\n",angle);
   }
   */
   
   
   Jive jive(80, 60);
   //cout<<"i'm here"<<endl;
  jive.start();
//    namedWindow( "Binary", WINDOW_NORMAL );
   while (!done) {
      if (getkey() == 'q') 
         done = true;
      usleep(100000);
   }

err_exit:
   jive.stop();
}
