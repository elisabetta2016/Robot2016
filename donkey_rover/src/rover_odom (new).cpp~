#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "libRover.h"
Rover rover(true);
int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  float x = 0.0;
  float y = 0.0;
  float th = 0.0;
  float v;
  float omega;

  /* ROVER INITIALIZATION */
  
  
  /*
  int retCode = rover.init();
  if (retCode != 0)
		{
     	// Failed to init the rover
		return -1;
  }
  */
  
  float vx = 0.1;
  float vy = -0.1;
  float vth;
  

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
//printf(" speed is %f ",vx);
  ros::Rate r(10.0);
  while(n.ok()){
    Time timestamp;
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    // MY ADDED CODE
    int retCode = rover.getSpeedVO(timestamp, v, omega);
    printf(" retCode  is %d, speed is  %f \n ",retCode,v);
    if (retCode != 0)
		{
     	// Failed to read the rover speed
		ROS_INFO("Failed to read the rover speed");
		return -1;
    }
    vth = omega;
    vx = v * cos(th);    
    vy = v * sin(th);


    //compute odometry in a typical way given the velocities of the robot
    float dt = (current_time - last_time).toSec();
    float delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    float delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    float delta_th = omega * dt;
//DEBUG
    delta_x = vx * dt;
    delta_y = vy * dt;
// DEBUG END
    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
