#include "pid_control.h"

double ref_angle;
double Pgain,Igain,Dgain;
double Perror,Perror_prev,Ierror,Derror;

extern int fd1;

void initialize_pid(double new_ref_angl){
	Pgain = Igain = Dgain =1;
	Perror = Perror_prev = Ierror = Derror = 0;
	ref_angle = new_ref_angl;
}

double compute_error(){
	double curr_angl,tot_error;
	curr_angl = get_magneto(fd1);
	Perror = ref_angle - curr_angl;
	if ((Perror>5) || (Perror<-5)){
	    Ierror += Perror;
	    if (Ierror >50){
		    Ierror = 50;
	    }
	    if (Ierror < -50){
		    Ierror = -50;
	    }
	    Derror = Perror - Perror_prev;
	    Perror_prev = Perror ;
	    tot_error = Perror*Pgain*1 + Ierror*Igain*1 + Derror*Dgain*1;
	}
	else{
	  tot_error=0;
	}
	return tot_error;
}

void controller(double tot_error){
	 int DUT;
  
	if (tot_error > PWM_period){
		tot_error = PWM_period-1;
	}
	else if (tot_error < -1*PWM_period){
		tot_error = -1*PWM_period+1;
	}
	
	if (tot_error < 0)
	{
		tot_error = -1*tot_error;
		DUT = (int)tot_error;
		if (DUT > PWM_period-1)
		{
			DUT = PWM_period-1;
		}
		take_right(DUT);
		
	}
	else if (tot_error > 0)
	{
			DUT = (int)tot_error;
			if (DUT > PWM_period-1)
			{
				DUT = PWM_period-1;
			}
		take_left(DUT);	
	}
	else if (tot_error == 0)
	{
		continue_straight();
	}
}
