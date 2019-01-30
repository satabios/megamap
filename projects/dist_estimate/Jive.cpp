/*! 
 * ============================================================================
 *
 * @addtogroup		jive
 * @{
 *
 * @file		jive.cpp
 * @version		1.0
 * @date		12/14/2015
 *
 * @note		Hand tracking class
 * 
 * Copyright(c) 2007-2012 Texas Instruments Corporation, All Rights Reserved.
 * TI makes NO WARRANTY as to software products, which are supplied "AS-IS"
 *
 * ============================================================================
 */
#define __JIVE_CPP__
#include "Jive.h"
#include <climits>
#include <algorithm>
#include <fstream>
//#include "motor.h"

//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////

volatile int eventCounter1 = 0;
volatile int eventCounter2 = 0;
volatile int eventCounter3 = 0;
volatile int eventCounter4 = 0;

int fd1;

#define encoder1_pin 4
#define encoder2_pin 5
#define encoder3_pin 21
#define encoder4_pin 7

void encoder_init();

void myInterrupt1(void);
void myInterrupt2(void);
void myInterrupt3(void);
void myInterrupt4(void);

void myInterrupt1(void) {
   eventCounter1++;
}

void myInterrupt2(void) {
   eventCounter2++;
}

void myInterrupt3(void) {
   eventCounter3++;
}

void myInterrupt4(void) {
   eventCounter4++;
}

void encoder_init()
{
  
  if ( wiringPiISR (encoder1_pin, INT_EDGE_RISING, &myInterrupt1) < 0 ) {
      fprintf (stderr, "Unable to setup encoder ISR: %s\n", strerror (errno));
  }
  
  //if ( wiringPiISR (encoder2_pin, INT_EDGE_RISING, &myInterrupt2) < 0 ) {
  //   fprintf (stderr, "Unable to setup encoder ISR: %s\n", strerror (errno));
  //}
  
  if ( wiringPiISR (encoder3_pin, INT_EDGE_RISING, &myInterrupt3) < 0 ) {
      fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
  }
  
  //if ( wiringPiISR (encoder4_pin, INT_EDGE_RISING, &myInterrupt4) < 0 ) {
  //    fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
  //}
  
}

//////////////////////////////////////////////////////////////////////////////

double X_pos,Y_pos,prev_Xpos,prev_Ypos,dist,dist1,dist2, Perror = 0.0, tot_error = 0.0, newpwm1 = 30, newpwm3 = 25, Pgain = 1;

double X_refer=0,Y_refer=0,angl_refer=0.0,angl_new=0.0,angl_diff=0.0,angl_final=0.0;

#define max_pwm_period 50
#define min_pwm_period 10

//ofstream ofile;

void dist_covered(){

  dist1=((eventCounter1*32)/301.0);
  dist2=((eventCounter3*32)/301.0);
  Perror = (eventCounter1 - eventCounter3);
  tot_error = Perror*Pgain*1;
  
  if(tot_error >10){
	  tot_error = 10;
  }
  if(tot_error < -10){
	  tot_error = -10;
  }
  
  newpwm1 = 30 - tot_error;
  newpwm3 = 25 + tot_error;
  
  softPwmWrite(pwm1_pin,newpwm1);
  softPwmWrite(pwm3_pin,newpwm3);
  
  eventCounter1=0;
  eventCounter3=0;
  dist=(dist1+dist2)/2;
}

void update_pos(){
  /*
    double ang;
    ang=(get_magneto(fd1));
    if(ang>=359.0 && ang<=360.0){
      ang=0.0;
    }
    cout<<ang<<endl;
    ang=(ang*pi)/180.0;
    */
    dist_covered();
    X_pos=prev_Xpos+dist*cos(angl_refer);
    Y_pos=prev_Ypos+dist*sin(angl_refer);
    
    prev_Xpos=X_pos;
    prev_Ypos=Y_pos;
}

/////////////////////////////////////////////////////////////////////////////

int sockfd;
int n;
socklen_t len;
char sendline[1024];
struct sockaddr_in servaddr;

void init_udp(){
  
    sockfd=socket(AF_INET,SOCK_DGRAM,0);
    bzero(&servaddr,sizeof(servaddr));
    servaddr.sin_family=AF_INET;
    servaddr.sin_addr.s_addr=inet_addr("192.168.43.228");		//155
    servaddr.sin_port=htons(5035);
    len=sizeof(servaddr);
    
}

/////////////////////////////////////////////////////////////////////////////

std::string shr;
int f_count =0;
int avg = 5; //Give in frames
float depth_avg =0.0;

//robot status
volatile int status=0,command =0,flag=0,flag2=0,flag3 =0,flag4 = 0, esc_count=0;

queue l,c,r;

#define WIN_SIZE        5

Mat gray_prev=Mat::zeros(60, 80, CV_8U);
Mat gray_next=Mat::zeros(60, 80,CV_8U);


Jive::Jive()
{
   init();
}

Jive::Jive(int w, int h) : TOFApp(w, h)
{
   init();
}

Jive::~Jive()
{
}

void Jive::init()
{
   _ampGain = 10.0;
   _depthClip = 2.0;
   _ampClip = 0.01;
   _illum_power = 100U;
   _intg = 8U;
   setLoopDelay(20);

   start_straight();
   
   
   encoder_init();
   
   fd1=setup_magneto();
   
   X_pos=0.0;
   Y_pos=0.0;
   prev_Xpos=0.0;
   prev_Ypos=0.0;
   
   dist=0.0;
   
  status = 2;
   
  init_udp();

}


void Jive::clipBackground(DepthFrame &in, DepthFrame &out)
{
   out.size = in.size;
   out.depth.clear();          
   out.amplitude.clear();
   for (int i = 0; i < in.depth.size(); i++) {
      out.depth.push_back((in.depth[i] < _depthClip && in.amplitude[i] > _ampClip) ? in.depth[i] : 0.0);
      out.amplitude.push_back((in.depth[i] < _depthClip && in.amplitude[i] > _ampClip) ? 1.0 : 0.0);
   }
}

void Jive::findKeyPoints(vector<cv::Point> &contour, vector<int> &hulls, vector<int> &defects, int depth)
{  
   vector<Vec4i> convDef;

   hulls.clear();
   defects.clear();   
   convexHull(Mat(contour), hulls, false ); 
   convexityDefects(contour, hulls, convDef);
   for (int k=1; k<convDef.size(); k++) {  // first defect is false  
      if (convDef[k][3] > depth*256) {
         int ind = convDef[k][2];
         defects.push_back(ind);
      }
   }
}

void Jive::clipBackground_WT(DepthFrame &in, DepthFrame &out)
{
   out.size = in.size;
   out.depth.clear();          
   out.amplitude.clear();
   for (int i = 0; i < in.depth.size(); i++) {
      out.depth.push_back((in.depth[i] < _depthClip && in.amplitude[i] > _ampClip) ? in.depth[i] : 0.0);
      out.amplitude.push_back((in.depth[i] < _depthClip && in.amplitude[i] > _ampClip) ? in.amplitude[i] : 0.0);
   }
}

void drawOptFlowMap (const Mat& flow, Mat& cflowmap, int step, const Scalar& color) {
 for(int y = 0; y < cflowmap.rows; y += step)
    for(int x = 0; x < cflowmap.cols; x += step)
    {
	const Point2f& fxy = flow.at< Point2f>(y, x);
	line(cflowmap, cv::Point(x,y), cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),color);
	circle(cflowmap, cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y)), 1, color, -1);
    }
}

float Jive::depthAt(DepthFrame frm, cv::Point p)
{
   return frm.depth[p.y*frm.size.height+p.x];
}


int Jive::adjPix(int pix)
{
   int out = pix*getDim().width/TOF_WIDTH;
   return (out<=0)?1:out;
}
void writeMatToFile(const char* filename, cv::Mat m)
{
   ofstream fout(filename);
   if(!fout){
       cout<<"File Not Opened"<<endl;  return;
   }

   for(int i=0; i<m.rows; i++){
       for(int j=0; j<m.cols; j++){
           fout<<m.at<float>(i,j)<<",";
       }
       fout<<"\n";
   }
   fout.close();
}

bool Jive::findminDepth(vector<cv::Point> &contour, cv::Point &center, float &mindepth)
{
   bool rc = false;
   float depth=20.0;
   cv::Point depthpoint = cv::Point(0, 0); 
   for (int x=0; x < depth_clipped.cols; x++) {
      for (int y=0; y < depth_clipped.rows; y++) {
         cv::Point p = cv::Point(x, y);
         if (pointPolygonTest(contour, p, false) >= 0) {
            float val = depth_clipped.at<float>(y, x);
            if(depth > val){
	      depth = val;
	      depthpoint = p;
	    }
         }
      }
   }
   if (depth != 20.0) { 
      mindepth = depth;
      center = depthpoint;
      rc = true;
   }
   return rc;
}

void Jive::update(DepthFrame *frm)
{

  DepthFrame hand;
    std::stringstream ss;
    int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
  double fontScale = 0.2;
  int thickness = 1;
  if(k==1){
      k++;
      clipBackground_WT(*frm, hand);
      _binaryMat = Mat(hand.size.height, hand.size.width, CV_32FC1, hand.amplitude.data());  
      _binaryMat.convertTo(gray_prev, CV_8U, 255.0);
  }
 
  
 // Initialising
  double max,min;
  Mat amp_clipped_rect,depth_clipped_rect,amp_clipped_rect_jpg,depth_clipped_rect_jpg, amp_big, depth_big; 
  amp_clipped_rect = Mat::zeros( frm->size.height,frm->size.width, CV_8U);
  depth_clipped_rect = Mat::zeros( frm->size.height,frm->size.width, CV_8U);
  amp_clipped_rect_jpg = Mat::zeros( frm->size.height,frm->size.width, CV_8U);
  depth_clipped_rect_jpg = Mat::zeros( frm->size.height,frm->size.width, CV_8U);

  
  // To view clipped image
  DepthFrame background_clipped;
  
  amp_clipped_jpg = Mat::zeros( frm->size.height,frm->size.width, CV_8U);
  depth_clipped_jpg = Mat::zeros( frm->size.height,frm->size.width, CV_8U);
  clipBackground_WT(*frm, background_clipped);
  amp_clipped = Mat(background_clipped.size.height,background_clipped.size.width, CV_32FC1, background_clipped.amplitude.data());
  depth_clipped = Mat(background_clipped.size.height,background_clipped.size.width, CV_32FC1, background_clipped.depth.data()); 
  if(!(amp_clipped.empty())){
  minMaxLoc(amp_clipped, &min, &max);
  amp_clipped.convertTo(amp_clipped_jpg, CV_8U, 255.0/max);

   }
   
  if(!(depth_clipped.empty())){
  minMaxLoc(depth_clipped, &min, &max);
  depth_clipped.convertTo(depth_clipped_jpg, CV_8U, 255.0/max);

  }
  Mat gray;
  vector< vector<cv::Point> > contours;
  vector<cv::Point>  approxcontour;
  vector<Vec4i> hierarchy;
  RNG rng(12345);
  Mat drawing_big;
  clipBackground(*frm, hand);
  _binaryMat = Mat(hand.size.height, hand.size.width, CV_32FC1, hand.amplitude.data());  
  _binaryMat.convertTo(gray, CV_8U, 255.0);
  threshold(gray, gray, 200, 255, THRESH_BINARY);
  findContours(gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
  Mat drawing = Mat::zeros( gray.size(), CV_8UC3 );
  int hands = 0;
  
  cv::Point center = cv::Point(0,0);
  float radius = 0;
    
  float depth_min = 0;
  Rect rect;
  float minim_depth=0;
  
   if (contours.size() > 0) {
      for( int i = 0; i < contours.size(); i++ ) {  
         if (contourArea(contours[i]) > adjPix(1000)) {
	    if(findminDepth(contours[i], center, depth_min))
	    depth_avg = depth_avg + depth_min;
	    else
	      cout<<"error"<<endl;
	    f_count++;
	    if(f_count == avg)
	    {
	      minim_depth = (depth_avg/avg);
	      //cout<<"minim_depth:  "<<minim_depth<<endl;
	      f_count = 0;
	      depth_avg = 0;
	    }

	    rect = boundingRect(contours[i]);
	    if(rect.area() > (int)400){
	      rectangle(drawing, rect.tl(),rect.br(),Scalar(255,255,255),2,8,0);
	      //cout<<"Width: "<<rect.width<<"Length:  "<<rect.height<<endl;
	    }
            }

      }
   }


  
  
  Mat flow_big=Mat::zeros(4*60, 4*80, CV_8U);
  //Optical Flow
  Mat flow;
  Mat flow_split[2];
  Mat magnitude = Mat::zeros( frm->size.height,frm->size.width, CV_8U);
  Mat angle;
  amp_clipped.convertTo(gray_next, CV_8U, 255.0);
  calcOpticalFlowFarneback(gray_prev, gray_next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
  Mat cflow;
  cvtColor(gray_prev, cflow, CV_GRAY2BGR);

  split(flow, flow_split);
  cartToPolar(flow_split[0], flow_split[1], magnitude, angle, false);

float myMAtMean1,myMAtMean2,myMAtMean3;
  
Scalar tempVal1 = mean(magnitude(cv::Rect(0,0,30,60)));
myMAtMean1 = tempVal1.val[0];

if(cvIsInf(myMAtMean1) || myMAtMean1<0.1) 
    myMAtMean1 = 0.0;
  

Scalar tempVal2 = mean(magnitude(cv::Rect(31,0,20,60)));
myMAtMean2 = tempVal2.val[0];
if(cvIsInf(myMAtMean2)|| myMAtMean2<0.1) 
    myMAtMean2 = 0.0;
  
Scalar tempVal3 = mean(magnitude(cv::Rect(50,0,30,60)));
myMAtMean3 = tempVal3.val[0];
if(cvIsInf(myMAtMean3)|| myMAtMean3<0.1) 
    myMAtMean3 = 0.0;

if (myMAtMean1==0 && myMAtMean2==0 && myMAtMean3==0){

	command =0;
}
else{
  if(myMAtMean1 < myMAtMean2)
  {
	if(myMAtMean1 < myMAtMean3){
	  command =4;
	}
	else{
	  command = 6;
	}
  }
  else
  {
	if(myMAtMean2 < myMAtMean3){

	  if(myMAtMean3 <myMAtMean1){
	  command = 6;
	  }
	  else{
	   command = 4;
	  }
	}
	else{
	  command = 6;
	}
  }
  if(flag == 0){
      angl_new = get_magneto(fd1);
      if(angl_new>=359.0 && angl_new<=360.0){
	angl_new=0.0;
      }
	printf("angl_new:: %f\n",angl_new);
      angl_new=(angl_new*pi)/180.0;
      flag =1;
  }
}

if ((myMAtMean1!=0 && myMAtMean2!=0 && myMAtMean3!=0) || (myMAtMean1==0 && myMAtMean2!=0 && myMAtMean3==0) || (myMAtMean1!=0 && myMAtMean2==0 && myMAtMean3!=0)){
	flag3 = 1;
	eventCounter1=0;
	eventCounter3=0;
}

if(flag3 == 1){
  stop_robot();
  take_right(70);
  delay(125);
  flag4=1;
  esc_count++;
  eventCounter1=0;
  eventCounter3=0;
}
else{
  switch(status){
  case 2:
		  switch(command){
				  case 4:
					  stop_robot();
					  take_left(70);
					  if(flag4==1){
					    esc_count++;
					  }
					  delay(125);
					  status = 1;
					  eventCounter1=0;
					  eventCounter3=0;
					  break;
				  case 6:
					  stop_robot();
					  take_right(70);
					  if(flag4==1){
					    esc_count++;
					  }
					  delay(125);
					  status = 3;
					  eventCounter1=0;
					  eventCounter3=0;
					  break;	
				  case 0:
					/*
					  angl_refer = get_magneto(fd1);
					  if(angl_refer>=359.0 && angl_refer<=360.0){
					    angl_refer=0.0;
					  }
					  //cout<<angl_refer<<endl;
					  angl_refer=(angl_refer*pi)/180.0;
					*/
					  update_pos();
					  esc_count=0;
					  flag4=0;
					  break;
		  }
		  break;
		  
  case 1:
		  switch(command){
				  case 4:
					    stop_robot();
					    take_left(70);
					    if(flag4==1){
					      esc_count++;
					    }
					    delay(125);
					    status = 1;
					    eventCounter1=0;
					    eventCounter3=0;
					  break;
				  case 6:
					    stop_robot();
					    take_right(70);
					    if(flag4==1){
					      esc_count++;
					    }
					    delay(125);
					    status = 3;
					    eventCounter1=0;
					    eventCounter3=0;
					  break;

				  case 0:
					  angl_final = get_magneto(fd1);
					  if(angl_final>=359.0 && angl_final<=360.0){
					    angl_final=0.0;
					  }
					  angl_final=(angl_final*pi)/180.0;
					  angl_diff = (angl_final - angl_new);
					  angl_refer = angl_refer +angl_diff;
					  flag = 0;
					  status=2;
					  continue_straight();
					  esc_count=0;
					  flag4=0;
					  eventCounter1=0;
					  eventCounter3=0;
					  break;
		  }
		  break;
  case 3:
		  switch(command){
				  case 4:
					    stop_robot();
					    take_left(70);
					    if(flag4==1){
					      esc_count++;
					    }
					    delay(125);
					    status = 1;
					    eventCounter1=0;
					    eventCounter3=0;
					  break;
				  case 6:
					    stop_robot();
					    take_right(70);
					    if(flag4==1){
					      esc_count++;
					    }
					    delay(125);
					    status = 3;
					    eventCounter1=0;
					    eventCounter3=0;
					  break;
				  case 0:
					  angl_final = get_magneto(fd1);
					  if(angl_final>=359.0 && angl_final<=360.0){
					    angl_final=0.0;
					  }
					  angl_final=(angl_final*pi)/180.0;
					  angl_diff = (angl_final - angl_new);
					  angl_refer = angl_refer +angl_diff;
					  flag = 0;
					  status=2;
					  continue_straight();
					  esc_count=0;
					  flag4 = 0;
					  eventCounter1=0;
					  eventCounter3=0;
					  break;
		  }
		  break;
  }
  
}

if(esc_count>14){
  take_right(300);
  esc_count=0;
printf("escape\n");
  flag4 = 0;
}

flag3 =0;
	
//cout<<myMAtMean1<<"\t"<<myMAtMean2<<"\t"<<myMAtMean3<<"\t"<<X_pos<<"\t"<<Y_pos<<"\t"<<rect.tl().y<< "\t"<<rect.tl().x<< "\t" <<rect.br().y<< "\t" <<rect.br().x<<"\t"<<(angl_refer*180.0)/pi<<endl;
cout<<"\n"<<myMAtMean1<<"\t"<<myMAtMean2<<"\t"<<myMAtMean3<<"\t"<<X_pos<<"\t"<<Y_pos<<"\t"<<(angl_refer*180.0)/pi<<"\t"<<Perror<<endl;
//cout<<"angles:: "<<(angl_refer*180.0)/pi<<"\t"<<(angl_new*180.0)/pi<<"\t"<<(angl_final*180.0)/pi<<endl;
sprintf(sendline,"%f %f %d %d %d %d %f %f\n\r",X_pos,Y_pos, rect.br().x, rect.br().y, rect.tl().x, rect.tl().y, minim_depth,(angl_refer*180.0)/pi);
//sprintf(sendline,"%f %f",X_pos,Y_pos);
sendto(sockfd,sendline,MAXLINE,0,(struct sockaddr*)&servaddr,len);
gray_next.copyTo(gray_prev);

}




#undef __JIVE_CPP__
/*! @} */
