#include <ros/ros.h>
#include "donkey_rover/Scanner_Command.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string.h>
#include <iostream>
#include <tf.h>



int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher scanner_pub = n.advertise<donkey_rover::Scanner_Command>("scanner_commands", 1);

  ros::Rate loop_rate(10);
  ROS_INFO("Welcome to scanner command wizard");
  char Scanner_Command[10];
  printf("\nEnter the scanner command, available commands: DoHoming, GoHome, Start, Stop:\n");
  scanf("%s",Scanner_Command);
  double Scanner_Ajustment_Angle;
  printf("\nEnter Scanner Adjustment angle (rad), -100 for no change:\n");
  scanf("%f",&Scanner_Ajustment_Angle);
  double Scanner_Roll_Angle; 
  printf("\nEnter Scanner Roll angle (rad), -100 for no change:\n");
  scanf("%f",&Scanner_Roll_Angle);
  double Scanner_Home_Angle;  
  printf("\nEnter Scanner Home angle (rad), -100 for no change:\n");
  scanf("%f",&Scanner_Home_Angle);
  int count = 0;
  double Scanner_Period;
  printf("\nEnter Scanner Period (s), -100 for no change:\n");
  scanf("%f",&Scanner_Period);
  

  while (ros::ok())
  {
    donkey_rover::Scanner_Command command_;
    
    
    //ROS_INFO("New Message Input");

    command_.Scanner_Command = Scanner_Command;
    

    command_.Scanner_Ajustment_Angle = Scanner_Ajustment_Angle;


    command_.Scanner_Roll_Angle = Scanner_Roll_Angle;   


    command_.Scanner_Home_Angle = Scanner_Home_Angle;   
   

    command_.Scanner_Period = Scanner_Period;          
   


    scanner_pub.publish(command_);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
    ROS_INFO("Message Sent, Number of messages that has been sent so far: %d",count);
  }


  return 0;
}

