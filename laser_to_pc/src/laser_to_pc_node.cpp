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
			SubFromScannerInfo_		= n_.subscribe("/Scanner_angle_sync", 1, &LaserToPcClass::scanner_msg_callback,this);
			SubFromLaser_			= n_.subscribe("scan", 1, &LaserToPcClass::laser_call_back,this);
			
			Ts = 3.00;
			Assemble_stage = 1;
			
			// publishers
			cloud_pub_ 			 = n_.advertise<sensor_msgs::PointCloud2> ("cloud_in", 1);
			Laser_cloud_pub_ 		 = n_.advertise<sensor_msgs::PointCloud2> ("laser_cloud_in", 1);
			
    			// services
    			client_ = n_.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
    			
    			// Initializers
			Ts = 3.00;
			Assemble_stage = 1;
			assemble_process = true; 
	
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
			temp(2) = scan_in->ranges[i] * sin(theta);    //2
			temp(0) = -1*scan_in->ranges[i] * cos(theta); //0 
			temp(1) = 0.0;				      //1
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
		//if (id == 1) ROS_INFO_ONCE("fisrt thread");
		//if (id == 2) ROS_INFO_ONCE("second thread");
		//if (id == 3) ROS_INFO_ONCE("third thread");
	
	}
	
	void laser_call_back(const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_2 (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_3 (new pcl::PointCloud<pcl::PointXYZ>);

		boost::thread_group tgroup;
		tgroup.create_thread (boost::bind (&LaserToPcClass::partial_pc,this,scan_in, 0.0f,     M_PI/3,   temp_cloud   ,1));
		tgroup.create_thread (boost::bind (&LaserToPcClass::partial_pc,this,scan_in, M_PI/3,   2*M_PI/3, temp_cloud_2 ,2));
		tgroup.create_thread (boost::bind (&LaserToPcClass::partial_pc,this,scan_in, 2*M_PI/3, M_PI,     temp_cloud_3 ,3));
		tgroup.join_all();
		*temp_cloud += *temp_cloud_2;
		*temp_cloud += *temp_cloud_3;
		
		
		
		
		//boost::thread first(&LaserToPcClass::partial_pc,scan_in,M_PI/3,M_PI,temp_cloud);
		//partial_pc(scan_in,M_PI/3,M_PI,temp_cloud);

		// sending cloud
		sensor_msgs::PointCloud2 cloud;
  		pcl::toROSMsg(*temp_cloud,cloud);
  		cloud.header.stamp = ros::Time::now();
  		cloud.header.frame_id = "/camera";
  		memcpy(&cloud.data[0], angle.b, 4);
  		
  		Laser_cloud_pub_.publish(cloud);	
	}
	
	void scanner_msg_callback(const std_msgs::Float32::ConstPtr& msg)
	{

		
		float Scanner_angle = msg->data;
		angle.f = Scanner_angle;
		
		
		transform = Eigen::Affine3f::Identity();
		
		transform.rotate (Eigen::AngleAxisf (Scanner_angle+M_PI, Eigen::Vector3f::UnitZ()));
		
		

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
		if (!assemble_process)
		{
			assemble_process = true;
			cloud_assemble();	
		}
		ros::MultiThreadedSpinner spinner(4);
		spinner.spin();
		//ros::spin ();
	
	}
  	

	protected:
	
	// Node Handler
	ros::NodeHandle n_;
		
	// Subscribers
	ros::Subscriber SubFromScannerInfo_;
	ros::Subscriber SubFromLaser_;
	
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

	
	
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_to_pc");
	ros::NodeHandle node;

	LaserToPcClass laser_to_pc (node);
	
	laser_to_pc.run();
	
	return 0;
}

	
		
