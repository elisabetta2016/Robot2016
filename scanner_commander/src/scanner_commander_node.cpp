#include <ros/ros.h>
#include "donkey_rover/Scanner_Command.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <math.h>

void imu_filtered_print(const sensor_msgs::Imu::ConstPtr& data)
{
   sensor_msgs::Imu msg = *data;
   //tf::Quaternion q(msg.orientation.x,msg.orientation.w,msg.orientation.z,msg.orientation.y);
   tf::Quaternion q(msg.orientation.y,msg.orientation.x,msg.orientation.w,msg.orientation.z);
   tf::Matrix3x3 m(q);
   double Roll, Pitch, Yaw;
   m.getRPY(Roll,Pitch,Yaw);
   //ROS_WARN("Roll:%4f, Pitch:%4f, Yaw:%4f ",Roll,Pitch,Yaw);
   //ros::Duration(1).sleep();
}

void imu_print(const sensor_msgs::Imu::ConstPtr& data)
{
   sensor_msgs::Imu msg = *data;
   tf::Quaternion q(msg.orientation.x,msg.orientation.w,msg.orientation.z,msg.orientation.y);
   //tf::Quaternion q(msg.orientation.y,msg.orientation.x,msg.orientation.w,msg.orientation.z);
   tf::Matrix3x3 m(q);
   double Roll, Pitch, Yaw;
   m.getRPY(Roll,Pitch,Yaw);
   float q0 = msg.orientation.w;
   float q1 = msg.orientation.x;
   float q2 = msg.orientation.y;
   float q3 = msg.orientation.z;
   //ROS_INFO("Roll:%4f, Pitch:%4f, Yaw:%4f ",Roll,Pitch,Yaw);
   float psi=180/M_PI*atan(2*(q1*q2+q0*q3)/(2*pow(q0,2)+2*pow(q1,2)-1));
   ROS_INFO("Yaw is: %5f",psi);
   ros::Duration(1).sleep();
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Subscriber imu_filtered_sub = n.subscribe("imu/data",1,imu_filtered_print);
  ros::Subscriber imu_sub          = n.subscribe("imu/data_raw",1,imu_print);
  ros::spin();

  return 0;
}

