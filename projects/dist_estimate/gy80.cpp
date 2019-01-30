#include "gy80.h"

int flagx=0;
double comp_angl;

int setup_magneto(void){
	int k;
	uint8_t id;

	if((k=wiringPiI2CSetup(0x1E)) >= 0){
		printf("setup success.\n");
	}
	else{
		printf("Error while setting up.\n");
	}

	id=wiringPiI2CReadReg8(k,0x0A);
	
	if(id==0x48){
		printf("Found magneto GY-80.\n");
	}

	wiringPiI2CWriteReg8(k,0x00,0x38);	//18
	
	wiringPiI2CWriteReg8(k,0x02,0x00);

	return k;
}

double get_magneto(int k){
  char addr[6]={0x03,0x04,0x05,0x06,0x07,0x08};
  uint8_t array[6];
  int16_t mag[3];
  double angl;
  
   int j,i;
  for(j=0;j<6;j++){
    array[j] = wiringPiI2CReadReg8(k,addr[j]);
  }
  
  j=0;
  
  for(i=0;i<3;i++)
  {
	  mag[i] = ((array[j]*256) | array[j+1]);
	  //mag[i] = mag[i]*0.3;
	  j=j+2;
  }
  
  mag[0]=mag[0]-5+1;		
  mag[2]=mag[2]+279-6;	
  angl=(atan2(mag[2],mag[0])*180.0)/pi;
  
  
  if(angl <0){
	  angl=angl+360.0;
  }
 if(flagx == 0){
  	comp_angl = angl;
  	flagx =1;
 }
 angl = angl - comp_angl;
 if(angl<0){
	 angl = angl +360.0;
 } 
 //angl = 360.0 -angl;
 //delay(10);
  return angl;
}
