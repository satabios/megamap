#include "boostxl.h"

int setup_magneto()
{
	int k,id;
	if((k=wiringPiI2CSetup(0x68)) >= 0){
		printf("setup success.\n");
	}
	else{
		printf("Error while setting up.\n");
	}
	
	id=wiringPiI2CReadReg8(k,0x75);
	
	if(id==0x68){
		printf("Found MPU.\n");
	}
	wiringPiI2CWriteReg8(k,0x6b,0x80);
	
	wiringPiI2CWriteReg8(k,0x6b,0x08);	
	
	wiringPiI2CWriteReg8(k,0x37,0x02);
	
	if((k=wiringPiI2CSetup(0x0C)) >= 0){
		printf("setup success.\n");
	}
	else{
		printf("Error while setting up.\n");
	}
	
	id=wiringPiI2CReadReg8(k,0x00);
	
	if(id==0x48){
		printf("Found magneto.\n");
	}
	return k;
	
}

double get_magneto(int fd){
  char addr[6]={0x03,0x04,0x05,0x06,0x07,0x08};
  uint8_t array[6];
  int16_t mag[3];
  
  int j,i;
  double angl;
  
  wiringPiI2CWriteReg8(fd,0x0A,0x01);
  
  delay(25);
  
  for(j=0;j<6;j++){
	  array[j] = wiringPiI2CReadReg8(fd,addr[j]);
  }
  
  j=0;
  for(i=0;i<3;i++)
  {
	  mag[i] = ((array[j+1]*256) | array[j]);
	  mag[i] = mag[i]*0.3;
	  j=j+2;
  }
  
  //printf("%d %d\n",mag[0],mag[1]);
  
  mag[0]=mag[0]-(-11.5)-4.5;	//4.5
  mag[1]=mag[1]-(-6.5)-27;		//27
  
  angl=(atan2(mag[1],mag[0])*180.0)/pi;
/*	
  if(angl <0){
	  angl=angl+360.0;
  }
  */
  //wiringPiI2CWriteReg8(k,0x0A,0x01);
  
  return angl;
}

double avg_magneto(int val,int fd1){
  int  l;
   double sum=0.0;
   for(l=0;l<val;l++){
     sum=sum+get_magneto(fd1);
   }
   
   return (sum/val);
  
}
