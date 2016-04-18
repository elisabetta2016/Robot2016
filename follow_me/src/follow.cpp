#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Polygon.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char** argv){
    ros::init(argc, argv, "follow_navigation_goals");

    ros::NodeHandle n;


    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }
    bool flag = true;
    bool flag_move= false;
    int count = 0;
    tf::TransformListener listener;
    tf::Quaternion rot;
    double roll, pitch, yaw;
    geometry_msgs::Quaternion rot2;
    move_base_msgs::MoveBaseGoal goal;
    ros::Rate r(4);
    tfScalar pose_y = 0.0;
    tfScalar pose_x = 0.0;
    ros::Duration(5.0).sleep();
while(ros::ok()){


    tf::StampedTransform transformL;
    tf::StampedTransform transform2;
    tf::StampedTransform left_hand;
    tf::StampedTransform right_hand;

    //listener.waitForTransform("/base_link", "/left_knee_1",ros::Time(0), ros::Duration(3.0));
    try{
    listener.lookupTransform("/base_link", "/head_1",ros::Time(0), transformL);
    listener.lookupTransform("/map", "/odom",ros::Time(0), transform2);
    listener.lookupTransform("/left_hip_1", "/left_hand_1",ros::Time(0), left_hand);
    listener.lookupTransform("/right_hip_1", "/right_hand_1",ros::Time(0), right_hand);
    flag=true;
    }
    catch(tf::TransformException& ex){
      ROS_ERROR("Tf faild: %s \n", ex.what());
      flag=false;
      ROS_INFO("I DO NOT See you!!!!!");
    }
    
    tfScalar tfY = transformL.getOrigin().y() ;
    tfScalar tfX = transformL.getOrigin().x() ;

    tfScalar distance = sqrt((tfX*tfX) + (tfY*tfY));
    //tfScalar angle = atan2(tfY,tfX)*180/3.1415; degree
    tfScalar angle = atan2(tfY,tfX);

    //ROS_INFO("angle:  %f",angle);

    //set a security offset from the person
    tfScalar secOff = 1.0; //meter

    rot = transformL.getRotation();
    tf::Matrix3x3 m(rot);
    m.getRPY(roll, pitch, yaw);
    rot = tf::createQuaternionFromRPY(0.0, 0.0, angle);

    tf::quaternionTFToMsg(rot,rot2);



    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 0.0; //tfX;
    goal.target_pose.pose.position.y = 0.0;//tfY;
    goal.target_pose.pose.orientation = rot2;
    float yaw2=yaw*180/3.1415;




    //ROS_INFO("Hi Mohsen");
  //ROS_INFO("Sending goal to: x= %f y= %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);   





     //ROS_INFO("left hand z: %f",left_hand.getOrigin().z());
     //ROS_INFO("left hand y: %f",left_hand.getOrigin().y());
     //ROS_INFO("left hand x: %f",left_hand.getOrigin().x());

    
      ROS_INFO("Sending goal");
      ROS_INFO("User x: %f",tfX);
      ROS_INFO("User y: %f",tfY);
      ROS_INFO("current position x: %f",pose_x);
      ROS_INFO("current position y: %f",pose_y);
      ac.cancelAllGoals();
      ac.sendGoal(goal);
     // flag = false;
      ROS_INFO("The yaw is %f", yaw2);
      ac.waitForResult();
    // New lines 
    pose_x = transformL.getOrigin().x();
    ROS_INFO("You are %f meter far", pose_x);
        
    if(pose_x > 2.5){
        goal.target_pose.pose.position.x = pose_x - 2.5 + 0.5;
        goal.target_pose.pose.orientation.w = 1;
        ROS_INFO("I am coming ... !");
        ac.sendGoal(goal);
        ac.waitForResult();
    }
    // End



    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, Goal Achived");
        //ROS_INFO("The yaw is %f", yaw2);
        flag = true;}
    else ROS_INFO("Failed to move the rover or aborted");

    /*if(left_hand.getOrigin().y() > 0) {
        flag_move = true;
        ROS_INFO("Start following");
    }

    if(right_hand.getOrigin().y() > 0) {
        flag_move = false;
        ROS_INFO("Stop following");
    }


    if(count%4 == 0){
        pose_x = transformL.getOrigin().x();
        ROS_INFO("You are %f meter far", pose_x);
        
        if(pose_x > 3)
        goal.target_pose.pose.position.x = pose_x - 3 + 0.5;
        else
        goal.target_pose.pose.position.x = 0.0;//tfY;
        goal.target_pose.pose.orientation.w = 1;
        ROS_INFO("I am coming ... !");
	
        ac.sendGoal(goal);
        ac.waitForResult();
        //ros::Duration(1.0).sleep();
    }*/
    /*if(count%36 == 0){
        ROS_INFO("I am coming ... !");
        pose_y = transformL.getOrigin().y();
        pose_x = transformL.getOrigin().x();
        goal.target_pose.pose.position.x = pose_x; //tfX;
        goal.target_pose.pose.position.y = pose_y;//tfY;
        goal.target_pose.pose.orientation = rot2;
        ac.sendGoal(goal);
       // flag = false;
        ROS_INFO("The yaw is %f", yaw2);
        ac.waitForResult();
        if((distance < 2))
        {
            ac.cancelAllGoals();
            //flag = false;
            ROS_INFO("Dont wanna knock you off poor kid!!!");
        }
        ros::Duration(1.0).sleep();
    }*/



  r.sleep();
  count++;
  }
  return 0;
}

