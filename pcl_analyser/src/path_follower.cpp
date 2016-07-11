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

#include <Eigen/Dense> 
using namespace Eigen;

class PathfollowerClass
{	
	public:
	
	PathfollowerClass(ros::NodeHandle& node)
	{
		n_=node;
		SubFromPath_		= n_.subscribe("/Path_pso", 1, &PathfollowerClass::pathCallback,this);
		SubFromOdom_		= n_.subscribe("/odom",     1, &PathfollowerClass::OdomCallback,this);
		SubFromControllerSpeed_ = n_.subscribe("/speedfollow",     1, &PathfollowerClass::speedcallback,this);
		
		speed_pub_	  	  = n_.advertise<geometry_msgs::Vector3> ("body_error", 1);
		
		//Initializer
		new_path = false;
		Vx = 0.0;
		Vy = 0.0;
		Vth = 0.0;
		x_base_orig  = 0.0;
		y_base_orig  = 0.0;
		th_base_orig = 0.0;
		sub_goal_err = 0.08;
		goal_achieved = false;
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
	
	void speedcallback(const geometry_msgs::Vector3::ConstPtr& msg)
	{
		if(msg->z > 5.00) goal_achieved = true;
		else         goal_achieved = false;
	}
	
	void run()
	{
		double dt = 0.0; 
		
		ros::Time current_time, last_time;
		current_time = ros::Time::now();
		last_time = ros::Time::now();
		
		geometry_msgs::Vector3 body_error;
		ros::Rate rate(50.0);
		Vector2f body_temp;
		Vector2f IF_temp;
		Matrix2f Rot;
		Rot << 1,0,
		       0,1;
		
		while (ros::ok())
		{
			current_time = ros::Time::now();
			dt = (current_time - last_time).toSec();
			
			th_base_orig += Vth*dt;
			x_base_orig += Vx*dt;
			y_base_orig += Vy*dt;
			
			//Rot Matrix
			Rot(0,0) =  cos(th_base_orig);		Rot(0,1) =  sin(th_base_orig);
			
			Rot(1,0) = -sin(th_base_orig);		Rot(1,1) =  cos(th_base_orig);
			
			
			last_time = current_time;
			double dx,dy;
			if(new_path)
			{
								
				sub_goal_x = path.poses[path_counter].pose.position.x;
				sub_goal_y = path.poses[path_counter].pose.position.y;
				//debug
				//sub_goal_x = 5;
				//sub_goal_y = 3;
				//debug
				
				IF_temp(0) = sub_goal_x - x_base_orig;
				IF_temp(1) = sub_goal_y - y_base_orig;
				
				//Apply the rotation
				body_temp = Rot * IF_temp;
				
				body_error.x = -body_temp(1);   //-1*(sub_goal_y-y);
				body_error.y =  body_temp(0);   //(sub_goal_x-x);
				body_error.z =  0.0;
				
				if(path_counter == path_size)
				{
					new_path = false;
				}
				
				dx = sub_goal_x - x_base_orig;
				dy = sub_goal_y - y_base_orig;
				//ROS_INFO("sub_goal   x:%f,  y:%f", sub_goal_x,sub_goal_y);
				//ROS_INFO("body_orig  x:%f,  y:%f", x_base_orig,y_base_orig);
				//ROS_WARN(" ---      dx:%f, dy:%f", dx,dy); 
				//ROS_INFO("dx = %f, dy = %f", dx, dy);
				if(goal_achieved)
				{
					ROS_INFO("Sub_goal Achieved!");
					path_counter ++;
					
				}
				speed_pub_.publish(body_error);
				
			}
			else
			{
				body_error.x = 0.0;
				body_error.y = 0.0;
				body_error.z = 0.0;
				speed_pub_.publish(body_error);
			}
		ros::spinOnce();
		rate.sleep();
		}
		
	}
	
	private:
	// Node Handler
	ros::NodeHandle n_;
	//Subscribers
	ros::Subscriber SubFromPath_;
	ros::Subscriber SubFromOdom_;
	ros::Subscriber SubFromControllerSpeed_;
	//Publishers
	ros::Publisher speed_pub_;
	bool goal_achieved;
	bool new_path;	
	double Vx;
	double Vy;
	double Vth;
	double x_base_orig;
	double y_base_orig;
	double th_base_orig;
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
