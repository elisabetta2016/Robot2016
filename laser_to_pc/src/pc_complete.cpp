#include <ros/ros.h>
#include <ros/timer.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
//Sensor_msgs
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
//Standards
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <string>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
// Laser geometry
#include <laser_geometry/laser_geometry.h>
// Messages
#include "donkey_rover/Rover_Scanner.h"
#include "std_msgs/Float32.h"
// Service
#include "laser_assembler/AssembleScans.h"

#include <Eigen/Dense> 


union U_ANGLE
{
  float f;
  uint8_t b[4];
};

class LaserToPcClass
{
	public:
		
	LaserToPcClass(ros::NodeHandle& node)
	{
			// Node handle
			n_=node;

			//subscribers
			SubFromScannerangle_		= n_.subscribe("/Scanner_angle_sync", 1, &LaserToPcClass::scanner_angle_callback,this);
			SubFromScannerInfo_		= n_.subscribe("/RoverScannerInfo", 1, &LaserToPcClass::scanner_msg_callback,this);
			
			
			
			// publishers
			cloud_pub_ 			 = n_.advertise<sensor_msgs::PointCloud2> ("cloud_in", 1);
			
			
    			// services
    			client_ = n_.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
    			
    			// Initializers
			Ts = 3.00;
			Assemble_stage = 1;
			//assemble_process = true; 
	
	}
	
	void scanner_msg_callback(const donkey_rover::Rover_Scanner::ConstPtr& msg)
	{
		Scanner_roll_angle = msg->Scanner_roll_angle;
	}
	
	void scanner_angle_callback(const std_msgs::Float32::ConstPtr& msg)
	{

		
		float Scanner_angle = msg->data;
		
		angle.f = Scanner_angle;
		
		
		
		
		if( (fabs(Scanner_angle) < 0.012) && (Assemble_stage == 1))
		{
			ROS_INFO("1st Stage %f", Scanner_angle);
			assem_start = ros::Time::now();
			Assemble_stage ++;
			return;
    		
    		}
    		if( (fabs(Scanner_angle + Scanner_roll_angle) < 0.012) && (Assemble_stage == 2) )
		{
			ROS_INFO("2end Stage result:%f  angle:%f", Scanner_angle + Scanner_roll_angle,Scanner_angle);
			Assemble_stage ++;
			return;

    		}
    		if( (fabs(Scanner_angle - Scanner_roll_angle) < 0.012) && (Assemble_stage == 3))
		{
			ROS_INFO("3rd Stage result:%f  angle:%f", Scanner_angle - Scanner_roll_angle,Scanner_angle);
			Assemble_stage ++;
			return;

    		}
    		if((fabs(Scanner_angle) < 0.012) && (Assemble_stage == 4))
    		{
    			ROS_INFO("4th Stage");
    			assem_end = ros::Time::now();
    			srv.request.begin = assem_start;
    			srv.request.end = assem_end;
    			Assemble_stage = 1;
    			assemble_process = false;
    			cloud_assemble();
    			return;
    		}
    	}
		
	void cloud_outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
	{
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  		sor.setInputCloud (cloud_in);
  		sor.setMeanK (50);
  		sor.setStddevMulThresh (1.0);
  		sor.filter (*cloud_filtered);
	}
	
	void cloud_assemble()
  	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    		if (client_.call(srv))
    	   	{
      			ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.points.size()));
      		
      			sensor_msgs::convertPointCloudToPointCloud2(srv.response.cloud, input_cloud);
      			pcl::fromROSMsg (input_cloud, *temp_cloud);
      			cloud_outlier_removal(temp_cloud,temp_cloud);
      			pcl::toROSMsg(*temp_cloud,input_cloud);
      			cloud_pub_.publish(input_cloud);
    	   	}
    		else
    	   	{
      			ROS_ERROR("Error making service call\n") ;
    	   	}
  	}
  	
  	void run()
	{
		// For parameters
		//ros::NodeHandle n("~");


		ros::spin();
	}
  	

	protected:
	
	// Node Handler
	ros::NodeHandle n_;
		
	// Subscribers
	ros::Subscriber SubFromScannerInfo_;
	ros::Subscriber SubFromLaser_;
	ros::Subscriber SubFromScannerangle_;
	
	// Publishers
	ros::Publisher cloud_pub_;
	ros::Publisher Laser_cloud_pub_; 
	
	
	// Services
	ros::ServiceClient client_;
	laser_assembler::AssembleScans srv;
		
	//Class Global Variables
	int Assemble_stage;
	ros::Time assem_start;
	ros::Time assem_end; 
	sensor_msgs::PointCloud2 input_cloud;
	Eigen::Affine3f transform;
	U_ANGLE angle;

	double Ts;
	bool assemble_process;
	float Scanner_roll_angle; 
	
	
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pc_complete");
	ros::NodeHandle node;

	LaserToPcClass laser_to_pc (node);
	
	laser_to_pc.run();
	
	return 0;
}

	
		
