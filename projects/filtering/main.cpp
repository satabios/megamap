#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZI> ("test_pcd.pcd", *cloud);

  pcl::visualization::CloudViewer unfiltered_viewer ("Simple Cloud Viewer");
  
  unfiltered_viewer.showCloud(cloud);
  
  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  
  pcl::visualization::CloudViewer filtered_viewer ("filtered Cloud Viewer");
  filtered_viewer.showCloud(cloud_filtered);
  
  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  pcl::visualization::CloudViewer filtered_viewer2 ("filtered2 Cloud Viewer");
  filtered_viewer.showCloud(cloud_filtered);
  
  while(1);
  
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZI> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZI> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

  return (0);
}
