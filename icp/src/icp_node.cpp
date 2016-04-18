#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Imu.h> 
#include "donkey_rover/Rover_Track_Speed.h"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_broadcaster.h> // Tf should be listed in the dependencies 


class icpClass
{
	public:
		
		icpClass(ros::NodeHandle& node)
		{
			//node
			n_=node;
			//Subscribers
			subPointCloud_	= n_.subscribe("/camera/depth/points", 1, &icpClass::cloud_cb,this);
			subImu_ = n_.subscribe("imu/data", 1, &icpClass::imu_cb, this);
			subRoverTrackSpeed_ = n_.subscribe("RoverTrackSpeed", 1, &icpClass::roverTrackSpeed_cb, this);
			//Publishers TO BE DONE
		}

		void roverTrackSpeed_cb (const donkey_rover::Rover_Track_Speed::ConstPtr& rts_msg)
		{
		  rover_track_speed_msg_ = *rts_msg;
		  received_rover_track_speed = true;
		  if(received_imu)
		  	//If I receive a RoverTrackSpeed msg and I received an imu msg I calculate the movement
		  	calculatePosition(rover_track_speed_msg_->header.stamp);
		}

		void imu_cb (const sensor_msgs::Imu::ConstPtr& imu_msg)
		{
		  imu_msg_ = *imu_msg;
		  received_imu = true;
		  if(received_rover_track_speed)
		  	//If I receive an imu msg and I received a RoverTrackSpeed msg I calculate the movement
		  	calculatePosition(imu_msg_->header.stamp);
		}
		
		void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
		{

  		// TO be Done -- The system must calculate the speed out of two consecutive point clouds and transform a TF
  		// To be done later, taking to account an initial estimation from IMU
  		if (!cloud_process){
			cloud_process = true; // Cloud process started
  			//Converting sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
  			
  			pcl::PCLPointCloud2::Ptr pcl_pc2 (new pcl::PCLPointCloud2);
  			pcl_conversions::toPCL(*input,*pcl_pc2);


  			//Voxelgrid filtering
  			ROS_INFO("Start voxel filtering");
  			pcl::PCLPointCloud2 cloud_filtered;
  			pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  			sor.setInputCloud (pcl_pc2);
  			sor.setLeafSize (0.1, 0.1, 0.1);
  			sor.filter (cloud_filtered);

  			// Convert data type PointCloud2 -> PointCloud<pcl::PointXYZ> 
  			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
 		 	pcl::fromPCLPointCloud2(cloud_filtered,*cloud_in);
  
  			//Creating the cloud_out which is only a shifted cloud_in
  			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  			// Filtering out NAN values
  			std::vector<int> mapping;
  			pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, mapping);
			
			//Saving the first Cloud _ to be run only once
  			if (first_exe) {
				*last_cloud = cloud_in;
				first_exe = false;
			}
  			
  			double vector[4], result[4];

  			for(i = 0; i < 4; i++)
  				result[i] = 0;

  			if(flag) {
  				for(i = 0; i < last_cloud->points.size(); i++) {
  					vector[0] = last_cloud->points[i].x;
  					vector[1] = last_cloud->points[i].y;
  					vector[2] = last_cloud->points[i].z;
  					vector[3] = 1;
  					for (j = 0; j < 4; j++)
	  					for (k = 0; k < 4; k ++){
	  						result[j] += curr_matrix[j][k] * vector[k];
	  					}
	  				last_cloud->points[i].x = result[0];
  					last_cloud->points[i].y = result[1];
  					last_cloud->points[i].z = result[2];
  				}	
  				flag = false;
  			}


		  	//Start ICP
  			ROS_WARN("ICP Starts!");
  			pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  			icp.setInputSource(last_cloud);
  			icp.setInputTarget(cloud_in);
  			pcl::PointCloud<pcl::PointXYZ> Final;
  			icp.align(Final);
  			std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  			icp.getFitnessScore() << std::endl;
  			std::cout << icp.getFinalTransformation() << std::endl;
  			ROS_INFO("ICP Done!");
  			cloud_process = false;  //Cloud process flag, ready to analyse the next cloud
  			// Saving the last_cloud for the next iteration
			*last_cloud = cloud_in;
  			//pub.publish (output);
  		    }
		}

		void run()
		{
			ros::Rate loop_rate(rate);

			while (ros::ok())
			{
				//ROS_INFO_ONCE("ICP Started");

				//icp_handle();
				ros::spinOnce();
				loop_rate.sleep();
			}
		}

	protected:
		// NodeHandle
		ros::NodeHandle n_;
		
		// Subscribers
		ros::Subscriber subPointCloud_;
		ros::Subscriber imu_subscriber_;
		ros::Subscriber rover_track_speed_subscriber_;
	
		
		// Publishers - the transformation should be published 

		
		//Other variables
		bool cloud_process = false;
		float rate = 10.0;

	private:
		//last_cloud
		pcl::PCLPointCloud2::Ptr last_cloud (new pcl::PCLPointCloud2);	
		bool first_exe = false;
		bool received_imu = false;
		bool received_rover_track_speed = false;
		bool flag = false;

		int i, j, k;

		double matrix[4][4];
		double curr_matrix[4][4];
		double prev_matrix[4][4];
		for(i = 0; i < 4; i++)
			for(j = 0; j < 4; j++)
				prev_matrix[i][j] = 1.0;

		double speed_x = 0.0, speed_y = 0.0, speed_z = 0.0;
		double prev_time = 0.0, curr_time = 0.0, dt = 0.0;
		//Translation
		double Tx = 0.0, Ty = 0.0, Tz = 0.0;
		double prev_Tx = 0.0, prev_Ty = 0.0, prev_Tz = 0.0;
		//New position
		double curr_pos_x = 0.0, curr_pos_y = 0.0, curr_pos_z = 0.0;
		double curr_roll = 0.0, curr_pitch = 0.0, curr_yaw = 0.0;
		double dR = 0.0, dP = 0.0, dY = 0,0;
		//Previous position
		double prev_pos_x = 0.0, prev_pos_y = 0.0, prev_pos_z = 0.0;
		double prev_roll = 0.0, prev_pitch = 0.0, prev_yaw = 0.0;

		sensor_msgs::Imu imu_msg_;
		donkey_rover::Rover_Track_Speed rover_track_speed_msg_;

		void calculatePosition(const ros::Time& time) 
		{
			curr_time = time.toSec();
			dt = curr_time - prev_time;

			ROS_INFO("Calculating position");

			double frts = rover_track_speed_msg_.Front_Right_Track_Speed;
		    double flts = rover_track_speed_msg_.Front_Left_Track_Speed;
		    double rrts = rover_track_speed_msg_.Rear_Right_Track_Speed;
		    double rlts = rover_track_speed_msg_.Rear_Left_Track_Speed;
		    double speed_avg = (frts + flts + rrts + rlts) / 4;



		    tf::Matrix3x3 rot, u, finalRot;
		    tf::Quaternion q;
		    tf::quaternionMsgToTF(imu_msg_.orientation, q);
		    //Gets roll, pitch and yaw values from the quaternion
		    tf::Matrix3x3(q).getRPY(curr_roll, curr_pitch, curr_yaw);
		    //Gets the 3D rotational matrix
		    rot = tf::Matrix3x3(q);
		    u.setValue(speed_avg, 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		    rot *= u;
		    tf::Vector3 vect =  rot.getColumn(0);
		    //Gets the speed in x, y and z
		    speed_x = vect.getX();
		    speed_y = vect.getY();
		    speed_z = vect.getZ();

		    //Gets the translation Tx, Ty & Tz
		    Tx = prev_Tx + (speed_x * dt);
		    Ty = prev_Ty + (speed_y * dt);
		    Tz = prev_Tz + (speed_z * dt);

		    //Gets the delta of roll, pitch and yaw in this way delta = current angle - previous angle
		    dR = curr_roll - prev_roll;
		    dP = curr_pitch - prev_pitch;
		    dY = curr_yaw - prev_yaw;

		    finalRot.setRPY(dR, dP, dY);

		    tf::Vector3 vect1 =  finalRot.getColumn(0);
		    matrix[0][0] = vect1.getX();
		    matrix[1][0] = vect1.getY();
		    matrix[2][0] = vect1.getZ();
		    matrix[3][0] = 0.0;
		    tf::Vector3 vect2 =  finalRot.getColumn(1);
		    matrix[0][1] = vect2.getX();
		    matrix[1][1] = vect2.getY();
		    matrix[2][1] = vect2.getZ();
		    matrix[3][1] = 0.0;
		    tf::Vector3 vect3 =  finalRot.getColumn(2);
		    matrix[0][2] = vect3.getX();
		    matrix[1][2] = vect3.getY();
		    matrix[2][2] = vect3.getZ();
		    matrix[3][2] = 0.0;

		    matrix[0][3] = Tx;
		    matrix[1][3] = Ty;
		    matrix[2][3] = Tz;
		    matrix[3][3] = 1.0;

		    int index_i = 0, index_j = 0;
		    double sum = 0.0;

		    do {
		    	for(j = 0; j < 4; j++)
		    		sum	 += matrix[index_i][j] * prev_matrix[j][index_j];
		    	curr_matrix[index_i][index_j] = sum;
		    	sum = 0.0;
		    	index_j++;
		    	if(index_j == 4) {
		    		index_i++;
		    		index_j = 0;
		    	}
		    } while(index_i < 4);

		    ROS_INFO("Position calculated");

			received_imu = false;
			received_rover_track_speed = false;

			flag = true;

			saveCurrent();
		}

		void saveCurrent() 
		{
			prev_pos_x = curr_pos_x;
			prev_pos_y = curr_pos_y;
			prev_pos_z = curr_pos_z;
			prev_roll = curr_roll;
			prev_pitch = curr_pitch;
			prev_yaw = curr_yaw;
			prev_time = curr_time;
			prev_Tx = Tx;
			prev_Ty = Ty;
			prev_Tz = Tz;
			prev_matrix = curr_matrix;
		}
}

int
main (int argc, char** argv)
{
	ros::init(argc, argv, "icp");
	ros::NodeHandle node;

	icpClass icpNode(node);

	icpNode.run();
	return 0;


}
