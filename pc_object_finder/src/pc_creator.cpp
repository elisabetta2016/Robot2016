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
#include "std_msgs/Float32.h"
#include "donkey_rover/Rover_Scanner.h"
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
			SubFromScannerInfo_		= n_.subscribe("/RoverScannerInfo", 1, &LaserToPcClass::scanner_msg_callback,this);
			SubFromScannerangle_		= n_.subscribe("/Scanner_angle_sync", 1, &LaserToPcClass::scanner_angle_callback,this);
			SubFromLaser_			= n_.subscribe("scan", 10, &LaserToPcClass::laser_call_back,this);
			
			
			// publishers
			Right_cloud_pub_ 		 = n_.advertise<sensor_msgs::PointCloud2> ("Right_cloud", 1);
			Left_cloud_pub_ 		 = n_.advertise<sensor_msgs::PointCloud2> ("Left_cloud", 1);
			
			

    			// Initializers
			right_published = false;
			left_published  = false;
	
	}
	
	void partial_pc(const sensor_msgs::LaserScan::ConstPtr& scan_in, float theta_in,float theta_out,pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud, int id)
	{
		clock_t start = clock();
		float theta = theta_in;
		int start_index = (int) floor( (scan_in->angle_max - M_PI/2) /scan_in->angle_increment);
		start_index += (int) floor(theta_in/scan_in->angle_increment);
		int end_index = (int) floor( (scan_in->angle_max - M_PI/2) /scan_in->angle_increment);
		end_index += (int) floor(theta_out/scan_in->angle_increment) + 1;
		
		for (int i = start_index; i< end_index ;i++)
		{
			ROS_INFO_ONCE("Start index: %d, End index: %d",start_index ,end_index );
			pcl::PointXYZ point;
			Eigen::Matrix4f M;
			M = transform.matrix();
			Eigen::Vector4f temp;
			temp(0) = scan_in->ranges[i] * sin(theta);    //2
			temp(1) = 1*scan_in->ranges[i] * cos(theta); //0 
			temp(2) = 0.0;				      //1
			temp(3) = 1.0;
			temp = M * temp;
			
			point.x = temp(0);
			point.y = temp(1);
			point.z = temp(2);
			
			theta += scan_in->angle_increment;
			temp_cloud->points.push_back(point);
			
		}
		clock_t end = clock();
		ROS_WARN_ONCE("loop time %f",(float) (end - start)/CLOCKS_PER_SEC);
		cloud_outlier_removal(temp_cloud,temp_cloud);

	
	}
	
	void laser_call_back(const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);

		/*
		boost::thread_group tgroup;
		tgroup.create_thread (boost::bind (&LaserToPcClass::partial_pc,this,scan_in, 0.0f,     M_PI/3,   temp_cloud   ,1));
		tgroup.join_all();
		*/
		if (Scanner_angle_curr < 0)
		{
			partial_pc(scan_in, M_PI/2,     M_PI,   temp_cloud   ,1);
		
		}
		if (Scanner_angle_curr > 0)
		{
			partial_pc(scan_in,   0.0f,     M_PI/2, temp_cloud   ,1);
		}
		inc_cloud_2d = *temp_cloud;
	
		
	
	}
	
	void scanner_msg_callback(const donkey_rover::Rover_Scanner::ConstPtr& msg)
	{
		roll_anlge = fabs(msg->Scanner_roll_angle);
	}
	
	void scanner_angle_callback(const std_msgs::Float32::ConstPtr& msg)
	{

		
		Scanner_angle_curr = msg->data;

		

		
		if (Scanner_angle_curr < 0.0f)
		{
			inc_cloud_3d_right += inc_cloud_2d;
			if (fabs(Scanner_angle_curr+roll_anlge) < 0.002 && !right_published)
			{
				sensor_msgs::PointCloud2 cloud_r;
  				pcl::toROSMsg(inc_cloud_3d_right,cloud_r);
  				cloud_r.header.stamp = ros::Time::now();
  				cloud_r.header.frame_id = "/laser";
  				
  				Right_cloud_pub_.publish(cloud_r);
  				//ROS_INFO("Right cloud");
  				inc_cloud_3d_right.clear();
  				right_published = true;
  				left_published = false;
			}	
		
		}
		
		if (Scanner_angle_curr > 0.0f)
		{
			inc_cloud_3d_left += inc_cloud_2d;
			if (fabs(Scanner_angle_curr-roll_anlge) < 0.002 && !left_published)
			{
				sensor_msgs::PointCloud2 cloud_l;
  				pcl::toROSMsg(inc_cloud_3d_left,cloud_l);
  				cloud_l.header.stamp = ros::Time::now();
  				cloud_l.header.frame_id = "/laser";
  				
  				Left_cloud_pub_.publish(cloud_l);
  				//ROS_INFO("Left cloud");
  				inc_cloud_3d_left.clear();
  				right_published = false;
  				left_published = true;
			}
				
		
		}		
		
		transform = Eigen::Affine3f::Identity();
		transform.rotate (Eigen::AngleAxisf (Scanner_angle_curr, Eigen::Vector3f::UnitX()));
		
		
		
		Scanner_angle_last = Scanner_angle_curr;
		
    	}
		
	void cloud_outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
	{
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  		sor.setInputCloud (cloud_in);
  		sor.setMeanK (50);
  		sor.setStddevMulThresh (1.0);
  		sor.filter (*cloud_filtered);
	}
	

  	void run()
	{
		// For parameters
		//ros::NodeHandle n("~");

		ros::MultiThreadedSpinner spinner(4);
		spinner.spin();
		//ros::spin ();
	
	}
  	

	protected:
	
	// Node Handler
	ros::NodeHandle n_;
		
	// Subscribers
	ros::Subscriber SubFromScannerInfo_;
	ros::Subscriber SubFromScannerangle_;
	ros::Subscriber SubFromLaser_;
	
	// Publishers
	ros::Publisher Right_cloud_pub_;
	ros::Publisher Left_cloud_pub_; 
	
	
	// Services
		
	//Class Global Variables
	
	Eigen::Affine3f transform;
	float roll_anlge; 
	pcl::PointCloud<pcl::PointXYZ> inc_cloud_2d;
	pcl::PointCloud<pcl::PointXYZ> inc_cloud_3d_left;
	pcl::PointCloud<pcl::PointXYZ> inc_cloud_3d_right;
	float Scanner_angle_curr;
	float Scanner_angle_last;
	bool left_published;
	bool right_published;
	
	
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pc_creator");
	ros::NodeHandle node;

	LaserToPcClass laser_to_pc (node);
	
	laser_to_pc.run();
	
	return 0;
}

	
		
