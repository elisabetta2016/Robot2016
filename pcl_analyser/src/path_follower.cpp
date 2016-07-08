#include <ros/ros.h>
#include <ros/timer.h>
#include <math.h>
#include <iostream>
#include <string>

// TF
#include <tf/transform_listener.h>

//Messages
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>


class PathfollowerClass
{	
	public:
	
	PathfollowerClass(ros::NodeHandle& node)
	{
		n_=node;
		SubFromPath_		= n_.subscribe("/Path_pso", 1, &ObstacleDetectorClass::pathCallback,this);
		SubFromOdom_		= n_.subscribe("/odom",     1, &ObstacleDetectorClass::OdomCallback,this);
		
		speed_pub_	  	  = n_.advertise<geometry_msgs::Vector3> ("Path_sim", 1);
		
		//Initializer
		new_path = false;
		Vx = 0.0;
		Vy = 0.0;
		Vth = 0.0;
		x = 0.0;
		y = 0.0;
		
	}
	
	void pathCallback(const nav_msgs::Path::ConstPtr& msg)
	{
		ROS_INFO("OBSTACLE AVOIDANCE ACTIVATED: new path received");
		path = *msg;
		new_path = true;
	}
	
	void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
	{
		Vx  = msg->twist.twist.linear.x;
		Vy  = msg->twist.twist.linear.y;
		Vth = msg->twist.twist.angular.z;
	
	}
	
	void run()
	{
		double dt = 0.0; 
		
		ros::Time current_time, last_time;
		current_time = ros::Time::now();
		last_time = ros::Time::now();
		while (ros::ok())
		{
			dt = (current_time - last_time).toSec();
			x += dt*Vx;
			y += dt*Vy;
			
		
		}
		
	}
	
	private:
	
	bool new_path;	
	double Vx;
	double Vy;
	double Vth;
	double x;
	double y;
	nav_msgs::Path path;
		
	
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_follower");
	ros::NodeHandle node;
	
	

	PathfollowerClass path_follower(node);
	
	path_follower.run();
	
	return 0;
}
