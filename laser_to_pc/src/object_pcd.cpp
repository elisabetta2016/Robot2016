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
#include <pcl/io/pcd_io.h>
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
#include "donkey_rover/Scanner_Command.h"

// Service
#include "laser_assembler/AssembleScans.h"

#include <Eigen/Dense> 


union U_ANGLE
{
  float f;
  uint8_t b[4];
};

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2D (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3D (new pcl::PointCloud<pcl::PointXYZ>);

class LaserToPcClass
{
	public:
		
	LaserToPcClass(ros::NodeHandle& node)
	{
			// Node handle
			n_=node;

			//subscribers
			SubFromScannerInfo_		= n_.subscribe("RoverScannerInfo", 1, &LaserToPcClass::scanner_msg_callback,this);
			SubFromLaser_			= n_.subscribe("scan", 1, &LaserToPcClass::laser_call_back,this);

			
			// publishers

			Laser_cloud_pub_ 		 = n_.advertise<sensor_msgs::PointCloud2> ("/Object_pcl", 1);
			Scanner_command_pub_		 = n_.advertise<donkey_rover::Scanner_Command> ("/scanner_commands", 1);
			
    			// services
    			
    			
    			// Initializers
    			Scanner_angle_last = 0.0;
			
			Assemble_stage = 1;
			range = 2.3;
			
	
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
			if(scan_in->ranges[i] > range )
			{
				theta += scan_in->angle_increment;
				continue;
			}
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
		//ROS_WARN_ONCE("loop time %f",(float) (end - start)/CLOCKS_PER_SEC);
		cloud_outlier_removal(temp_cloud,temp_cloud);

	
	}
	
	void laser_call_back(const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{
		

		/*boost::thread_group tgroup;
		tgroup.create_thread (boost::bind (&LaserToPcClass::partial_pc,this,scan_in, M_PI/2,     M_PI,   cloud_2D   ,1));
		tgroup.join_all();*/
		partial_pc(scan_in, M_PI/2,     M_PI,   cloud_2D   ,1);

	
	}
	
	void scanner_msg_callback(const donkey_rover::Rover_Scanner::ConstPtr& msg)
	{

		float Scanner_angle = msg->Scanner_angle;
		if (fabs(Scanner_angle - Scanner_angle_last) > 0.0001)
		{
		
			if(Assemble_stage < 4)  *cloud_3D += *cloud_2D;
		  		
			// sending cloud
		
			
  			pcl::toROSMsg(*cloud_3D,cloud);
  			cloud.header.stamp = ros::Time::now();
  			cloud.header.frame_id = "/laser";
  			Laser_cloud_pub_.publish(cloud);
		}
		
		transform = Eigen::Affine3f::Identity();
		transform.rotate (Eigen::AngleAxisf (Scanner_angle, Eigen::Vector3f::UnitX()));    		
    		Scanner_angle_last = Scanner_angle;
    		//ROS_INFO("stage: %d, angle: %f",Assemble_stage,Scanner_angle);
    		
    		if ((Scanner_angle < -M_PI/6) && Assemble_stage==1) 
    		{
    			Assemble_stage++;
    			ROS_INFO("First stage");	
    		}
    		if ((Scanner_angle > -0.001) && Assemble_stage==2) 
    		{
    			Assemble_stage++;
    			ROS_INFO("Second stage");
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
	
  	
  	void run()
	{
		// For parameters
		//ros::NodeHandle n("~");
		donkey_rover::Scanner_Command scanner_msg;
		scanner_msg.Scanner_Command = "Start";
		scanner_msg.Scanner_Ajustment_Angle = 0.0;
		scanner_msg.Scanner_Roll_Angle = M_PI/2;
		scanner_msg.Scanner_Period = 10.0;
		
		ros::Rate loop_rate(10);
		while(ros::ok())
		{
			if (fabs(Scanner_angle_last + M_PI/2) < 0.03 )
			{
				scanner_msg.Scanner_Command = "GoHome";
			
			}
			
			if (Assemble_stage == 3) 
			{
				ROS_INFO("Object scanned successfully");
				ROS_WARN("The Cloud continues to be published");
				Assemble_stage++;	
				//pcl::PCDWriter writer;
				//writer.write<pcl::PointXYZ> ("model.pcd", *cloud_3D, false); //src/laser_to_pc/models/
			}
			if(Assemble_stage == 4){
		  	
		  		cloud.header.stamp = ros::Time::now();
  				cloud.header.frame_id = "/laser";
  				Laser_cloud_pub_.publish(cloud);
			
			}
			
			if(Assemble_stage < 4) Scanner_command_pub_.publish(scanner_msg);
			ros::spinOnce();
			loop_rate.sleep();
		}

		
	
	}
  	

	protected:
	
	// Node Handler
	ros::NodeHandle n_;
		
	// Subscribers
	ros::Subscriber SubFromScannerInfo_;
	ros::Subscriber SubFromLaser_;
	
	// Publishers
	ros::Publisher Laser_cloud_pub_; 
	ros::Publisher Scanner_command_pub_; 
	
	// Services

		
	//Class Global Variables
	int Assemble_stage;
	Eigen::Affine3f transform;
	float Scanner_angle_last;
	float range;
	sensor_msgs::PointCloud2 cloud;
	
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_pcd");
	ros::NodeHandle node;

	LaserToPcClass laser_to_pc (node);
	
	laser_to_pc.run();
	
	return 0;
}

	
		
