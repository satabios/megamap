#include "Jive.h"

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
   
   //start_straight();
   //delay(2000);
   //stop_robot();
   //delay(2000);
  //take_left(8000);
  //stop_robot();
   //delay(2000);
   //take_right(8000);
   //stop_robot();
   //delay(2000);
    //while(1); 
//   
   
   /*
   double angle;
   int fd1;
   fd1=setup_magneto();
   while(1){
     
    //angle=avg_magneto(8,fd1);
     angle=get_magneto(fd1);
    delay(100);
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
