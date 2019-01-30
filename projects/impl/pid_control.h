#include "boostxl.h"
#include "motor.h"

#define PWM_period 60

void initialize_pid(double);
double compute_error();
void controller(double);

