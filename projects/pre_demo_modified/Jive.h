/*! 
 * ============================================================================
 *
 * @addtogroup		jive
 * @{
 *
 * @file		jive.h
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
#include "TOFApp.h"
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "motor.h"
#include "boostxl.h"
///////////////////////////////////////////////

#include<sys/types.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<string.h>
#include<arpa/inet.h>
#include<string.h>
#include<arpa/inet.h>
#include<stdio.h>

#define MAXLINE 1024

///////////////////////////////////////////////

#ifndef __JIVE_H__
#define __JIVE_H__

class Jive : public TOFApp
{
public:
  int k=0;
   Jive();
   Jive(int w, int h);
   ~Jive();
   void init();
   void update(DepthFrame *frm);

private:
   Mat _binaryMat;
   Mat _depthMat;
   float _ampGain;
   float _depthClip;
   float _ampClip;
   uint _illum_power;
   uint _intg;
      Mat amp_clipped,depth_clipped,amp_clipped_jpg,depth_clipped_jpg,cam_depth_eq;

private:
   void clipBackground(DepthFrame &in, DepthFrame &out);
   void clipBackground_WT(DepthFrame &in, DepthFrame &out);
   bool findPalmCenter(vector<cv::Point> &contour, cv::Point &center, float &radius);
   bool findminDepth(vector<cv::Point> &contour, cv::Point &center, float &mindepth);
   void findKeyPoints(vector<cv::Point> &contour, vector<int> &hull, vector<int> &defects, int depth);
   void distillHullPoints(vector<cv::Point> &contour, vector<int> &hull, vector<int>&rhull, float maxDist);
   int  findMedianHull(vector<cv::Point> &contour, vector<int> hull);
   void kCurvature(vector<cv::Point> &contour, vector<int> &hull, int kmin, int kmax, double ang, vector<int> &tips);
   float depthAt(DepthFrame frm, cv::Point p);
   int  adjPix(int pix);
};

class queue
{
              float queue1[3];
              int rear,front;
      public:
              queue()
                {
                     rear=-1;
                     front=-1;
                }
              void insert(int x)
               {
                   if(rear >  2)
                    {
                       //cout <<"queue over flow";
                       front=rear=-1;
                       return;
                    }
                    queue1[++rear]=x;
                    //cout <<"inserted" <<x;
               }
              void delet()
               {
                   if(front==rear)
                     {
                         //cout <<"queue under flow";
                         return;
                     }
                     //cout <<"deleted" <<queue1[++front];
                }
              void display()
               {
                   if(rear==front)
                     {
                          //cout <<" queue empty";
                          return;
                     }
                   for(int i=front+1;i<=rear;i++)
                   cout <<queue1[i]<<" ";
               }
};

#endif // __JIVE_H__
/*! @} */

