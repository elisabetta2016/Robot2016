#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  ros::Publisher rover_vel =
    node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transformL;
    tf::StampedTransform transformR;
    try{
    	listener.waitForTransform("/base_link", "/left_knee_1",
    	                          ros::Time(0), ros::Duration(3.0));

    	listener.lookupTransform("/base_link", "/left_knee_1",
    	                                       ros::Time(0), transformL);
    	listener.lookupTransform("/base_link", "/right_knee_1",
    	                                       ros::Time(0), transformR);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    //Calculation of the middle between knees

    tfScalar tfY = (transformL.getOrigin().y() + transformR.getOrigin().y()) / 2.0;
	tfScalar tfX = (transformL.getOrigin().x() + transformR.getOrigin().x()) / 2.0;

   // ROS_INFO("%f\n",tfY);
    //ROS_INFO("%f\n",tfX);

	//Scale factor of linear and angular velocity(???)
	tfScalar scaleLin = 1.0;
	tfScalar scaleAng = 5.0;

    geometry_msgs::Twist vel_msg;

    tfScalar currentDistance = sqrt(pow(tfX, 2) +pow(tfY, 2));
    tfScalar goalDistance = 1.0;

    tfScalar currentAngle = atan2(tfY,tfX);

    ROS_INFO("%f\n",currentDistance);
    ROS_INFO("%f\n",currentAngle);


    //If the robot is not correct oriented, it can only rotate but not move forward  and backward


    vel_msg.angular.z = -scaleAng * currentAngle;
    vel_msg.linear.x = scaleLin * ( currentDistance - goalDistance );
    
    
    //If the robot is correct oriented,it can move forward and backward.
    //I stimate an angular error of |0.2| rad
 

    

    ROS_INFO("linear velocity %f\n", vel_msg.linear.x);
    ROS_INFO("angular velocity%f\n",vel_msg.angular.z);


    rover_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};
