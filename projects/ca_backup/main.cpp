#include "Jive.h"
#include "motor.h"

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

volatile int eventCounter1 = 0;

void myInterrupt1(void) {
   eventCounter1++;
}

int main(int argc, char *argv[])
{
   wiringPiSetup();
   
   
   if ( wiringPiISR (encoder1_pin, INT_EDGE_RISING, &myInterrupt1) < 0 ) {
	fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
	return 1;
    }
   
   motor1_start();
   motor2_start();
   int key;
   bool done = false;
   Mat bImg;
   Jive jive(80, 60);
   
   jive.start();
//    namedWindow( "Binary", WINDOW_NORMAL );
   while (!done) {
     
      printf( "%d\n", eventCounter1 );
      eventCounter1 = 0;
      delay( 1000 );
     
      if (getkey() == 'q') 
         done = true;
      usleep(100000);
   }

err_exit:
   jive.stop();
}

