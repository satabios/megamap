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
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/impl/passthrough.hpp>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/impl/statistical_outlier_removal.hpp>
//#include "opencv2/flann/flann.hpp"



pcl::PointCloud<pcl::PointXYZI> cloud;		//(80,60,(0,0,0,0))
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(&cloud);
//pcl::visualization::CloudViewer unfiltered_viewer ("Simple Cloud Viewer");

// pcl::PointCloud<pcl::PointXYZI> filt_cloud;		//(80,60,(0,0,0,0))
// pcl::PointCloud<pcl::PointXYZI>::Ptr filt_cloud_ptr(&filt_cloud);
 pcl::visualization::CloudViewer filtered_viewer ("filtered Cloud Viewer");



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
  cloud.width    = 80;
  cloud.height   = 60;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);
  /*
  filt_cloud.width    = 80;
  filt_cloud.height   = 60;
  filt_cloud.is_dense = false;
  filt_cloud.points.resize (filt_cloud.width * filt_cloud.height);
  */
  
   _ampGain = 10.0;
   _depthClip = 0.6;
   _ampClip = 0.01;
//    _illum_power = 100U;
//    _intg = 8U;
   setLoopDelay(20);
//    namedWindow( "Binary", WINDOW_NORMAL );
//    namedWindow( "Contours", WINDOW_NORMAL );
}




float Jive::depthAt(DepthFrame frm, cv::Point p)
{
   return frm.depth[p.y*frm.size.height+p.x];
}



void Jive::update(XYZIPointCloudFrame *frm)	//XYZIPointCloudFrame
{
  std::ofstream f("point.bin", std::ios::binary | std::ios::out);
  f.write((char *)frm->points.data(), sizeof(IntensityPoint)*frm->points.size());
//f.write((char *)frm->depth.data(), sizeof(float)*frm->size.width*frm->size.height);
  
  for (size_t i = 0; i < cloud.points.size(); ++i)
  {
    cloud.points[i].x = frm->points[i].x;
    cloud.points[i].y = frm->points[i].y;
    cloud.points[i].z = frm->points[i].z;
    cloud.points[i].intensity = frm->points[i].i;
  }
  
  //unfiltered_viewer.showCloud (cloud_ptr);
  
/*  
   pcl::PassThrough<pcl::PointXYZI> pass;
   pass.setInputCloud (cloud_ptr);
   


   pass.setFilterFieldName ("intensity");
   pass.setFilterLimits(0.01, 10.0);
   //pass.setFilterLimitsNegative (true);
   pass.filter(*cloud_ptr);		//filt_cloud_ptr
 
   pass.setFilterFieldName ("y");
   pass.setFilterLimits(-10.0, 10.0);
   //pass.setFilterLimitsNegative (true);
   pass.filter(*cloud_ptr);		//filt_cloud_ptr*/
   /*
  pass.setFilterFieldName ("z");
  pass.setFilterLimits(0.0, 10.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter(*cloud_ptr);		//filt_cloud_ptr*/
  /*
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud(cloud_ptr);
  sor.setMeanK(10);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_ptr);
  */
    // f.write((char *)cloud_ptr->points.data(), sizeof(IntensityPoint)*frm->points.size());
  
  filtered_viewer.showCloud(cloud_ptr);
  
  
  pcl::io::savePCDFileASCII("test_two_objects_pcd.pcd",cloud);
  std::cerr<<"saved "<<cloud.points.size()<<" data points"<<std::endl;
  
  
  //viewer.~CloudViewer();
   while (!filtered_viewer.wasStopped ())
   {
     
    }
  
   
   }




#undef __JIVE_CPP__
/*! @} */
