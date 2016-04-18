#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

bool flag = true;
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

  // TO be Done -- The system must calculate the speed out of two consecutive point clouds and transform a TF
  // To be done later, taking to account an initial estimation from IMU
  if (flag){

  //Converting sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
  ROS_INFO("Saving PointCloud Started");
  pcl::PCLPointCloud2::Ptr pcl_pc2 (new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*input,*pcl_pc2);
  std::cout << "PointCloud before filtering: " << pcl_pc2->width * pcl_pc2->height 
       << " data points (" << pcl::getFieldsList (*pcl_pc2) << ")." <<'\n';
  
  //Writing PointCloud from topic, before filtering
  pcl::PCDWriter writer;
  writer.write ("input_beforeFiltering.pcd", *pcl_pc2);
  ROS_WARN("Unfiltered PointCloud Saved Successfully");
  //Voxelgrid filtering
  ROS_INFO("Start voxel filtering");
  pcl::PCLPointCloud2 cloud_filtered;
  pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
  vg.setInputCloud (pcl_pc2);
  vg.setLeafSize (0.05, 0.05, 0.05);
  vg.filter (cloud_filtered);

  // Convert data type PointCloud2 -> PointCloud<pcl::PointXYZ> 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(cloud_filtered,*cloud_in);
  
  ROS_INFO("1");

  //Creating the cloud_out which is only a shifted cloud_in
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  /*std::cout << "Saved " << cloud_in->points.size () << " data points to input:"  << std::endl;*/
  ROS_INFO("2");

  // Filtering out NAN values
  std::vector<int> mapping;
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, mapping);

  //New Output Point
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ended (new pcl::PointCloud<pcl::PointXYZ>);

  //StatisticalOutlierRemoval filter
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_in);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_ended);

  std::cout << "PointCloud after STATISTICAL OUTLIER REMOVE filtering: " << cloud_ended->width * cloud_ended->height 
       << " data points (" << pcl::getFieldsList (*cloud_ended) << ")."<<'\n';

  //Saving PointCloud after filtering  
  writer.write ("input_afterFiltering.pcd", *cloud_ended);
  ROS_WARN("Filtered PointCloud Saved Successfully");
  flag = false;
  ros::shutdown();
  // Publish the data.
  //pub.publish (output);
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "icp");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/assembled_cloud", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  //ros::Duration(1).sleep();
  // Spin
  ros::spin();
  if (flag==false) ros::shutdown();
}
