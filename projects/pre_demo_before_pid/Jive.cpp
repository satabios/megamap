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
  
  if ( wiringPiISR (encoder2_pin, INT_EDGE_RISING, &myInterrupt2) < 0 ) {
      fprintf (stderr, "Unable to setup encoder ISR: %s\n", strerror (errno));
  }
  
  if ( wiringPiISR (encoder3_pin, INT_EDGE_RISING, &myInterrupt3) < 0 ) {
      fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
  }
  
  if ( wiringPiISR (encoder4_pin, INT_EDGE_RISING, &myInterrupt4) < 0 ) {
      fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
  }
  
}

//////////////////////////////////////////////////////////////////////////////

double X_pos,Y_pos,prev_Xpos,prev_Ypos,dist,dist1,dist2;

//ofstream ofile;

void dist_covered(){
  
  //cout<<eventCounter1<<endl;
  dist1=((eventCounter1*31.4)/302.0);
  dist2=((eventCounter3*31.4)/302.0);
  eventCounter1=0;
  eventCounter3=0;
  dist=(dist1+dist2)/2.0;

}

void update_pos(){
    double ang;
    //ang=(avg_magneto(8,fd1));
    ang=(get_magneto(fd1));
    cout<<ang<<endl;
    ang=(ang*pi)/180.0;
    dist_covered();
    X_pos=prev_Xpos+dist*cos(ang);
    Y_pos=prev_Ypos+dist*sin(ang);
     //cout<<ang<<endl;
    /*
    if (ang<0){
	 if(ang>-1.57){
	   X_pos=prev_Xpos+dist*cos(ang);
	   Y_pos=prev_Ypos+dist*sin(ang);
	 }
	 else{
	  X_pos=prev_Xpos+dist*cos(ang);
	  Y_pos=prev_Ypos+dist*sin(ang);
	 }
    }
    else{
	if(ang>1.57){
	  X_pos=prev_Xpos+dist*cos(ang);
	  Y_pos=prev_Ypos+dist*sin(ang);
	}
	else{
	  X_pos=prev_Xpos+dist*cos(ang);
	  Y_pos=prev_Ypos+dist*sin(ang);  
	}
    }*/
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
    servaddr.sin_addr.s_addr=inet_addr("192.168.43.155");		//228
    servaddr.sin_port=htons(5035);
    len=sizeof(servaddr);
    
}

/////////////////////////////////////////////////////////////////////////////

std::string shr;
int f_count =0;
int avg = 1; //Give in sec
float depth_avg =0.0;

//robot status
int status=0,command =0,flag=0,flag2=0;
queue l,c,r;

#define WIN_SIZE        5
//#define pi 		3.14159265358979323846
//the address of variable which receives trackbar position update

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
//    namedWindow( "Binary", WINDOW_NORMAL );
//    namedWindow( "Contours", WINDOW_NORMAL );
   //motor1_start();
   //motor2_start();
   
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
  //ofile.open("coordinates.txt");
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

// bool Jive::findPalmCenter(vector<cv::Point> &contour, cv::Point &center, float &radius)
// {
//    bool rc = false;
//    float m10, m01, m00;
// 
//    m10 = m01 = m00 = 0;
//    for (int x=0; x < _binaryMat.cols; x++) {
//       for (int y=0; y < _binaryMat.rows; y++) {
//          cv::Point p = cv::Point(x, y);
//          if (pointPolygonTest(contour, p, false) >= 0) {
//             float val = _binaryMat.at<float>(y, x);
//             m00 += val;
//             m10 += x*val;
//             m01 += y*val;
//          }
//       }
//    }
//    if (m00 > 0.0) { 
//       center = cv::Point((int)(m10/m00), (int)(m01/m00));
//       float r2 = 2*getDim().width*getDim().width;
//       for (int i=0; i<contour.size(); i++) {
//          float x2 = (float)contour[i].x - center.x;
//          x2 *= x2;
//          float y2 = (float)contour[i].y - center.y;
//          y2 *= y2;
//          if (r2 > x2+y2)
//             r2 = x2+y2;    
//       }
//       radius = sqrt(r2);
//       rc = true;
//    }
//    return rc;
// }
bool Jive::findminDepth(vector<cv::Point> &contour, cv::Point &center, float &mindepth)
{
   bool rc = false;
   float depth=20.0;
   cv::Point depthpoint = cv::Point(0, 0); 
   for (int x=0; x < _depthMat.cols; x++) {
      for (int y=0; y < _depthMat.rows; y++) {
         cv::Point p = cv::Point(x, y);
         if (pointPolygonTest(contour, p, false) >= 0) {
            float val = _depthMat.at<float>(y, x);
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

// int Jive::findMedianHull(vector<cv::Point> &contour, vector<int> hull) 
// {
//    cv::Point centroid(0,0);
//    for (int i = 0; i < hull.size(); i++) {
//       centroid = centroid + contour[hull[i]];
//    }
//    centroid.x = centroid.x / hull.size();
//    centroid.y = centroid.y / hull.size();
// 
//    int rc = 0;
//    double minDist = 1e6;
//    for (int i = 0; i < hull.size(); i++) {
//       double dist = cv::norm(contour[hull[i]]-centroid);
//       minDist = (dist < minDist) ? dist : minDist;
//       rc = hull[i];
//    } 
//    return rc;
// }


// void Jive::distillHullPoints(vector<cv::Point> &contour, vector<int> &hull, vector<int>&rhull, float maxDist)
// {
//    int start = 0;
//    cv::Point p1, p2;
//    vector<int> temp;
// 
//    rhull.clear();
//    if (hull.size() > 2) {
//       // Find the starting point
//       for (int k=0; k < hull.size(); k++) {
//          int n = (k+1 >= hull.size()) ? k+1-hull.size() : k+1;
//          p1 = contour[hull[k]]; 
//          p2 = contour[hull[n]];
//          if (cv::norm(p1-p2) > (double)maxDist) {
//             start = n;
//             temp.clear();
//             temp.push_back(hull[n]);
//             break;
//          }
//       }
//    
//       // Walk around the entire hull points once from the starting point
//       if (temp.size() > 0) {
//          for (int i=start; i < start+hull.size(); i++) {
//             int k = (i >= hull.size()) ? i-hull.size() : i;
//             int n = (i+1 >= hull.size()) ? i+1-hull.size() : i+1;
//             p1 = contour[hull[k]];
//             p2 = contour[hull[n]];
//             if (cv::norm(p1-p2) > (double)maxDist) {
//                int m = findMedianHull(contour, temp);
//                rhull.push_back(m);
//                temp.clear();
//             }
//             temp.push_back(hull[n]);
//          }
//       } suggested by Gordon:
//    }
// }


// void Jive::kCurvature(vector<cv::Point> &contour, vector<int> &hull, int kmin, int kmax, 
//                       double ang, vector<int> &tips)
// {
//    tips.clear();
//    for (int i=0; i < hull.size(); i++) {
//       int n = hull[i];
//       cv::Point p = contour[n];
//       bool done = false;
//       for (int k = 1; k < kmax && !done; k++) {
//          cv::Point p1 = (n-k < 0) ? contour[contour.size()+(n-k)] : contour[n-k];
//          cv::Point p2 = (n+k > contour.size()) ? contour[n+k-contour.size()] : contour[n+k];
//          double dval = ((p1.x-p.x)*(p2.x-p.x)+(p1.y-p.y)*(p2.y-p.y)) 
//                      / (norm(Mat(p1), cv::Mat(p))*norm(Mat(p2), Mat(p)));
//          double a = acos(dval)*180.0/3.1415926;
//          if (a < ang && k >= kmin) {
//             tips.push_back(n);
//             done = true;
//          } 
//       } 
//    }
// }
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

void Jive::update(DepthFrame *frm)
{
//   std::ofstream f("point.bin", std::ios::binary | std::ios::out);
//   f.write((char *)frm->depth.data(), sizeof(float)*frm->size.width*frm->size.height);
//   
//   cout<<"here"<<endl;
  DepthFrame hand;
  if(k==1){
//       createButton("Caculate", callbackButton_calculate, NULL, CV_PUSH_BUTTON, 1);
      k++;
      clipBackground_WT(*frm, hand);
      _binaryMat = Mat(hand.size.height, hand.size.width, CV_32FC1, hand.amplitude.data());  
      _binaryMat.convertTo(gray_prev, CV_8U, 255.0);
  }
 
  
 // Initialising
  double max,min;
  Mat amp_clipped_rect,depth_clipped_rect,amp_clipped_rect_jpg,depth_clipped_rect_jpg; 
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
//   imshow("Background Segmented ", amp_clipped_jpg);
   }
   
  if(!(depth_clipped.empty())){
  minMaxLoc(depth_clipped, &min, &max);
  depth_clipped.convertTo(depth_clipped_jpg, CV_8U, 255.0/max);
  //imshow("Depth",depth_clipped_jpg);    //changed
  }
  
  Mat flow_big=Mat::zeros(4*60, 4*80, CV_8U);
  //Optical Flow
  Mat flow;
  Mat flow_split[2];
  Mat magnitude = Mat::zeros( frm->size.height,frm->size.width, CV_8U);
  Mat angle;
  Mat drawing = Mat::zeros( flow_big.size(), CV_8UC3 );
  amp_clipped.convertTo(gray_next, CV_8U, 255.0);
  calcOpticalFlowFarneback(gray_prev, gray_next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
  Mat cflow;
  cvtColor(gray_prev, cflow, CV_GRAY2BGR);
    drawOptFlowMap(flow, cflow, 10, CV_RGB(0, 255, 0));		//changed
    resize(cflow,flow_big,Size(240,320),CV_INTER_LINEAR);
    
//     imshow("Seg1", amp_clipped_jpg(Rect(0,0,30,60)));
  split(flow, flow_split);
  cartToPolar(flow_split[0], flow_split[1], magnitude, angle, false);
//   cv::Mat tmp = magnitude(cv::Rect(0,0,30,60));
//   seg1.copyTo(tmp);
//   cv::Mat tmp2 = magnitude(cv::Rect(31,0,20,60));
//   seg2.copyTo(tmp2);
//   Mat tmp3 = magnitude(cv::Rect(50,0,30,60));
//   seg3.copyTo(tmp3);
//   imshow("seg1",magnitude(cv::Rect(0,0,30,60)));
//   imshow("seg2",seg2);
//   imshow("seg3",seg3);
//   
  float myMAtMean1,myMAtMean2,myMAtMean3;
  
//   if (countNonZero(magnitude(cv::Rect(0,0,30,60)))){
//     Scalar tempVal1 = mean(magnitude(cv::Rect(0,0,30,60)));
//     myMAtMean1 = tempVal1.val[0];}
//   else{
//     myMAtMean1 = 0.0;
//   }
//   
//    if (countNonZero(magnitude(cv::Rect(31,0,20,60)))){
//     Scalar tempVal2 = mean(magnitude(cv::Rect(31,0,20,60)));
//     myMAtMean2 = tempVal2.val[0];}
//   else{
//     myMAtMean2 = 0.0;
//   }
//   
//   if (countNonZero(magnitude(cv::Rect(50,0,30,60)))){
//     Scalar tempVal3 = mean(magnitude(cv::Rect(50,0,30,60)));
//     myMAtMean3 = tempVal3.val[0];}
//   else{
//     myMAtMean3 = 0.0;
//   }
  
   /*
  if (!((magnitude(cv::Rect(0,0,30,60))).empty())){
    Scalar tempVal1 = mean(magnitude(cv::Rect(0,0,30,60)));
    myMAtMean1 = tempVal1.val[0];
    if(cvIsInf(myMAtMean1))
          myMAtMean1 = 0;
  }
  else{
    myMAtMean1 = 0.0;
  }
  
  if (!((magnitude(cv::Rect(31,0,20,60))).empty())){
    Scalar tempVal2 = mean(magnitude(cv::Rect(31,0,20,60)));
    myMAtMean2 = tempVal2.val[0];}
  else{
    myMAtMean2 = 0.0;
  }
//   writeMatToFile("CSV.csv",magnitude(cv::Rect(50,0,30,60)));
  if (!((magnitude(cv::Rect(50,0,30,60))).empty())){
    Scalar tempVal3 = mean(magnitude(cv::Rect(50,0,30,60)));
    
    myMAtMean3 = tempVal3.val[0];
    if(cvIsInf(myMAtMean3)) cout << "Here" << endl;
  }
  else{
    myMAtMean3 = 0.0;
  }*/
  
////////////////////////////////////////////////////////////////////////////////
   
 //update_pos();
////////////////////////////////////////////////////////////////////////////////
   
Scalar tempVal1 = mean(magnitude(cv::Rect(0,0,30,60)));
myMAtMean1 = tempVal1.val[0];
//cout<<magnitude(cv::Rect(0,0,30,60))<<endl;
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


//cout << magnitude(cv::Rect(50,0,30,60)) <<"\n"<<endl;

 
//cout<<"mean"<<myMAtMean3<<endl;
 /*
 if( !isfinite(myMAtMean1)){
   myMAtMean1 = 0.0;
 }
 
 if(!isfinite(myMAtMean2)){
   myMAtMean2 = 0.0;
 }
   
 if(!isfinite(myMAtMean3)){
   myMAtMean3 = 0.0;
 }*/
 
  
   
 //cout<<myMAtMean1<<"   "<<myMAtMean2<<"   "<<myMAtMean3<<"    "<<X_pos<<"   "<<Y_pos<<endl;
 
if (myMAtMean1==0 && myMAtMean2==0 && myMAtMean3==0){
	cv::putText(drawing,"straight",cv::Point(75,100),CV_FONT_HERSHEY_SIMPLEX,4,cv::Scalar(0,201,238),5);
// 	flag++;
// 	if(flag==5){
// 	  command =0;
// 	  flag=0;
// 	  flag2=0;
// 	}
	command =0;
}
else{
  if(myMAtMean1 < myMAtMean2)
  {
	if(myMAtMean1 < myMAtMean3){
	  cv::putText(drawing,"Left",cv::Point(75,100),CV_FONT_HERSHEY_SIMPLEX,4,cv::Scalar(0,201,238),5);
	  command =4;
	}
	else{
	  cv::putText(drawing,"Right",cv::Point(75,100),CV_FONT_HERSHEY_SIMPLEX,4,cv::Scalar(0,201,238),5);
	  command = 6;
	}
  }
  else
  {
	if(myMAtMean2 < myMAtMean3){
	  cv::putText(drawing,"Center",cv::Point(75,100),CV_FONT_HERSHEY_SIMPLEX,4,cv::Scalar(0,201,238),5);
	  //command = 5;
	  if(myMAtMean3 <myMAtMean1){
	  command = 6;
	  }
	  else{
	   command = 4;
	  }
	}
	else{
	  cv::putText(drawing,"Rstop_robot();ight",cv::Point(75,100),CV_FONT_HERSHEY_SIMPLEX,4,cv::Scalar(0,201,238),5);
	  command = 6;
	}
  }
}
/*
if (myMAtMean1!=0 && myMAtMean2!=0 && myMAtMean3!=0){
	move_back(300);
	eventCounter1=0;
}
*/
switch(status){
case 2:
		switch(command){
				case 4:
					stop_robot();
					//move_back(500);
					flag2=1;
					stop_robot();
					take_left(100);
					status = 1;
					eventCounter1=0;
					eventCounter3=0;
					break;
				case 6:
					stop_robot();
					//move_back(500);
					flag2=1;
					stop_robot();
					take_right(100);
					status = 3;
					eventCounter1=0;
					eventCounter3=0;
					break;	
				case 0:
					 update_pos();
					
					break;
		}
		break;
		
case 1:
		switch(command){
				case 4:
					//if (flag2 !=1){
					  stop_robot();
					  take_left(100);
					  status = 1;
					  eventCounter1=0;
					  eventCounter3=0;
					//}
					break;
				case 6:
					//if(flag2 != 1){
					  stop_robot();
					  take_right(100);
					  status = 3;
					  eventCounter1=0;
					  eventCounter3=0;
					//}
					break;

				case 0:
					status=2;
					continue_straight();
					eventCounter1=0;
					eventCounter3=0;
					break;
		}
		break;
case 3:
		switch(command){
				case 4:
					//if(flag2 !=1){
					  stop_robot();
					  take_left(100);
					  status = 1;
					  eventCounter1=0;
					  eventCounter3=0;
					//}
					break;
				case 6:
					//if(flag2 !=1){
					  stop_robot();
					  take_right(100);
					  status = 3;
					  eventCounter1=0;
					  eventCounter3=0;
					//}
					break;
				case 0:
					status=2;
					continue_straight();
					eventCounter1=0;
					eventCounter3=0;
					break;
		}
		break;
}
	
	
cout<<myMAtMean1<<"   "<<myMAtMean2<<"   "<<myMAtMean3<<"    "<<X_pos<<"   "<<Y_pos<<endl;

//ofile<<X_pos<<" "<<Y_pos<<endl;
sprintf(sendline,"%f %f",X_pos,Y_pos);
sendto(sockfd,sendline,MAXLINE,0,(struct sockaddr*)&servaddr,len);
//changed
//   imshow("Direction", drawing);
//    
//     imshow("OpticalFlowFarneback", flow_big);
//changed      
  
//   minMaxLoc(flow_split[0], &min, &max);
//   flow_split[0].convertTo(flow_split[0], CV_8U, 255/max);
// 
//   minMaxLoc(flow_split[1], &min, &max);
//   flow_split[1].convertTo(flow_split[1], CV_8U, 255/max);
  gray_next.copyTo(gray_prev);
  //  imshow("flow1", flow_split[0]);
  //  imshow("flow2", flow_split[1]);
}




#undef __JIVE_CPP__
/*! @} */
