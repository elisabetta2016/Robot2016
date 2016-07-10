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
#include <nav_msgs/Odometry.h>

class PathfollowerClass
{	
	public:
	
	PathfollowerClass(ros::NodeHandle& node)
	{
		n_=node;
		SubFromPath_		= n_.subscribe("/Path_pso", 1, &PathfollowerClass::pathCallback,this);
		SubFromOdom_		= n_.subscribe("/odom",     1, &PathfollowerClass::OdomCallback,this);
		
		speed_pub_	  	  = n_.advertise<geometry_msgs::Vector3> ("body_error", 1);
		
		//Initializer
		new_path = false;
		Vx = 0.0;
		Vy = 0.0;
		Vth = 0.0;
		x = 0.0;
		y = 0.0;
		sub_goal_err = 0.04;
	}
	
	void pathCallback(const nav_msgs::Path::ConstPtr& msg)
	{
		ROS_INFO("OBSTACLE AVOIDANCE ACTIVATED: new path received");
		path = *msg;
		new_path = true;
		path_size = path.poses.size();
		path_counter = 0;		

		
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
		
		geometry_msgs::Vector3 body_error;
		
		while (ros::ok())
		{
			current_time = ros::Time::now();
			dt = (current_time - last_time).toSec();
			x += dt*Vx;
			y += dt*Vy;
			last_time = current_time;
			double dx,dy;
			if(new_path)
			{
								
				sub_goal_x = path.poses[path_counter].pose.position.x;
				sub_goal_y = path.poses[path_counter].pose.position.y;
				//ROS_INFO("sub_goal   x:%f, y:%f", sub_goal_x,sub_goal_y);
				body_error.x = -1*sub_goal_y;
				body_error.y = sub_goal_x;
				body_error.z = 0.0;
				
				if(path_counter == path_size)
				{
					new_path = false;
				}
				dx = sub_goal_x - x;
				dy = sub_goal_y - y; 
				//ROS_INFO("dx = %f, dy = %f", dx, dy);
				if(fabs(dx) < sub_goal_err && fabs(dy) < sub_goal_err)
				{
					ROS_INFO("Sub_goal Achieved!");
					path_counter ++;
					
				}
				speed_pub_.publish(body_error);
				
			}
		ros::spinOnce();
		
		}
		
	}
	
	private:
	// Node Handler
	ros::NodeHandle n_;
	//Subscribers
	ros::Subscriber SubFromPath_;
	ros::Subscriber SubFromOdom_;
	//Publishers
	ros::Publisher speed_pub_;

	bool new_path;	
	double Vx;
	double Vy;
	double Vth;
	double x;
	double y;
	double sub_goal_x;
	double sub_goal_y;
	double sub_goal_err;	
	nav_msgs::Path path;
	int path_size;
	int path_counter;
	
		
	
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_follower");
	ros::NodeHandle node;
	
	

	PathfollowerClass path_follower(node);
	
	path_follower.run();
	
	return 0;
}
