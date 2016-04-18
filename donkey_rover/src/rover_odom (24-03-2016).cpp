#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "libRover.h"
#include <sensor_msgs/Joy.h>
#include <math.h>
#include "donkey_rover/Scanner_Command.h"
#include "donkey_rover/Rover_Track_Speed.h"
#include "donkey_rover/Rover_Track_Bogie_Angle.h"
#include "donkey_rover/Rover_Scanner.h"
#include "donkey_rover/Rover_Power_Data.h" 
#include <stdlib.h>
//#include <custom_msgs/velocityEstimate.h>

Rover rover(false);
EScannerState state;
float scannerRaw;
float scannerCal;
float VX = 0.0;
float VY = 0.0;
bool Low_Battery = false;

class DonkeyRoverClass
{
	public:
		
		DonkeyRoverClass(ros::NodeHandle& node)
		{
			n_=node;

			//subscribers
			subFromJoystick_ 		= n_.subscribe("joy", 1, &DonkeyRoverClass::joyCallback,this);
			subFromCMDVEL_ 			= n_.subscribe("cmd_vel", 1, &DonkeyRoverClass::CMDVELLCommander,this);
			subFromScannerCommander_ 	= n_.subscribe("c1", 10, &DonkeyRoverClass::ScannerCommander,this);
			//subFromIMUSpeed_= n_.subscribe("mti/filter/velocity", 10, &DonkeyRoverClass::imuspeed,this);
			subFromRightLeftCommands_	= n_.subscribe("speedfollow", 5, &DonkeyRoverClass::RLcommander,this);
			subFromscannerdata_		= n_.subscribe("scanner_data", 1, &DonkeyRoverClass::scannerRawValueSet,this);
			subFromscannercommands_		= n_.subscribe("scanner_commands", 1, &DonkeyRoverClass::SetScanner,this);

			// publishers
			odom_pub 	   	  = n_.advertise<nav_msgs::Odometry>("odom", 100);
			twist_pub 	  	  = n_.advertise<geometry_msgs::Twist>("twist", 100);
			Rover_Track_Speed_pub     = n_.advertise<donkey_rover::Rover_Track_Speed>("RoverTrackSpeed", 100);
			Rover_Track_Angles_pub    = n_.advertise<donkey_rover::Rover_Track_Bogie_Angle>("RoverTrackAngles", 100);	
			Rover_Scanner_Data_pub    = n_.advertise<donkey_rover::Rover_Scanner>("RoverScannerInfo", 100);
			Rover_Power_Data_pub      = n_.advertise<donkey_rover::Rover_Power_Data>("RoverPowerInfo", 100);
		}


		// Global Varriables of the Class


		// Subscriber callbacks
		/*
		void imuspeed(const custom_msgs::velocityEstimate::ConstPtr& speed){
    			custom_msgs::velocityEstimate s = *speed;
    			VX = s.velE;
    			VY = s.velN;
		}*/

		void RLcommander(const geometry_msgs::Vector3::ConstPtr& s)
		{
  			geometry_msgs::Vector3 speed = *s;
  			float VL = speed.y;
  			float VR = speed.x;
  			int retCode = rover.setSpeed (VL,VR);
  			if (retCode != 0)
    			 	{
     			   // Failed to set the speed
     			   ROS_ERROR("Failed to move the rover to follow");
  			 }
		}
		
		void scannerRawValueSet(const geometry_msgs::Vector3::ConstPtr& vector)
		{
		geometry_msgs::Vector3 new_speed = *vector;
		scannerRaw = new_speed.x;

		}
		
		void SetScanner(const donkey_rover::Scanner_Command::ConstPtr& msg)
		{
		donkey_rover::Scanner_Command scanner_command_msg = *msg;
		bool new_adjustment_angle;
		bool new_roll_angle;
		bool new_home_angle;
		bool new_scanner_period;
		bool new_scanner_command;
		
		if (abs(temp_adjustment_angle - scanner_command_msg.Scanner_Ajustment_Angle)<0.001) new_adjustment_angle=false;
		else {  new_adjustment_angle=true;
                ROS_INFO("New Adjustment Angle, %f",temp_adjustment_angle );}
		if (abs(temp_roll_angle - scanner_command_msg.Scanner_Roll_Angle)<0.001) new_roll_angle=false;
		else {  new_roll_angle=true;
                ROS_INFO("New roll Angle, %f", temp_roll_angle);}
		if (abs(temp_home_angle - scanner_command_msg.Scanner_Home_Angle)<0.001) new_home_angle=false;
		else {  new_home_angle=true;
                ROS_INFO("New home Angle");}
		if (abs(temp_scanner_period - scanner_command_msg.Scanner_Period)<0.001) new_scanner_period=false;
		else {  new_scanner_period=true;
                ROS_INFO("New Scanner Period, %f", temp_scanner_period);}
		if (temp_scanner_command == scanner_command_msg.Scanner_Command) new_scanner_command=false;
		else {  new_scanner_command=true;
                ROS_INFO("New Scanner Command");}
		

		int retCode;
			//Commands
			temp_scanner_command = scanner_command_msg.Scanner_Command;
 			 if (scanner_command_msg.Scanner_Command =="GoHome" && new_scanner_command ){
    				  while (state != ESSIdle)
     			 	  {
       					cout << '.' << flush;
       					Time::sleep(1, 0);
       					retCode = rover.getScannerState(state);
     				  }
                      retCode = rover.sendScannerCommand(ESCStop);
                        Time::sleep(1, 0);
     			 	  retCode = rover.sendScannerCommand(ESCGoHome);
      			 	  if (retCode != 0)
      		 	  	  {
        		  	       ROS_ERROR("Scanner GoHome failed");

      				  }
  			 } else if(scanner_command_msg.Scanner_Command == "Start" && new_scanner_command)
  			 {
     			 	while (state != ESSIdle)
      				{
       					cout << '.' << flush;
       					Time::sleep(1, 0);
       					retCode = rover.getScannerState(state);
      			 	}
      			 	retCode = rover.sendScannerCommand(ESCStart);
      			 	if (retCode != 0)
      			 	{
       		         		ROS_ERROR("Scanner Starting failed");

     				 }
  			} else if(scanner_command_msg.Scanner_Command == "Stop" && new_scanner_command)
  			{
      				retCode = rover.sendScannerCommand(ESCStop);
      				if (retCode != 0)
      				{
       					ROS_ERROR("Scanner Stopping failed");

      				}
  			} else if (scanner_command_msg.Scanner_Command == "DoHoming" && new_scanner_command)
  			{
     				retCode = rover.sendScannerCommand(ESCDoHoming);

  			}	  
  			//else ROS_ERROR("Scanner Command is not valid"); 
  			
			//Scanner Adjustment Angle
			if (scanner_command_msg.Scanner_Ajustment_Angle != -100 && new_adjustment_angle){
				  temp_adjustment_angle = scanner_command_msg.Scanner_Ajustment_Angle;
				  ROS_INFO("Start Set Scanner Adjustment Angle Process");
    				  while (state != ESSIdle)
     			 	  {
       					cout << '.' << flush;
       					Time::sleep(1, 0);
       					retCode = rover.getScannerState(state);
     				  }
     			 	  retCode = rover.setScannerAdjustment(temp_adjustment_angle);
      			 	  if (retCode != 0) ROS_ERROR("Set Scanner Adjustment Angle failed");
				  else ROS_INFO("Scanner Ajustment Angle is successfully set to %f",temp_adjustment_angle);	
			}
			//Scanner Roll Angle
			if (scanner_command_msg.Scanner_Roll_Angle != -100 && new_roll_angle){
				  temp_roll_angle = scanner_command_msg.Scanner_Roll_Angle;
				  ROS_INFO("Start Set Scanner Roll Angle Process");
    				  while (state != ESSIdle)
     			 	  {
       					cout << '.' << flush;
       					Time::sleep(1, 0);
       					retCode = rover.getScannerState(state);
     				  }
     			 	  retCode = rover.setScannerAngle(temp_roll_angle);
      			 	  if (retCode != 0) ROS_ERROR("Set Scanner Roll Angle failed");
				  else ROS_INFO("Scanner Roll Angle is successfully set to %f",temp_roll_angle);	
			}
			//Scanner Home Angle
			if (scanner_command_msg.Scanner_Home_Angle  != -100  && new_home_angle){
				  temp_home_angle = scanner_command_msg.Scanner_Home_Angle;
				  ROS_INFO("Start Set Scanner Home Angle Process");
    				  while (state != ESSIdle)
     			 	  {
       					cout << '.' << flush;
       					Time::sleep(1, 0);
       					retCode = rover.getScannerState(state);
     				  }
     			 	  //retCode = rover.setScannerHomePosition(temp_home_angle);
                      ROS_ERROR("Do Not change the home position, use the adjustment angle instead");
      			 	  if (retCode != 0) ROS_ERROR("Set Scanner Home Angle failed");
				  else ROS_INFO("Scanner Home Angle is successfully set to %f",temp_home_angle);	
			}
			//Scanner Period 
			if (scanner_command_msg.Scanner_Period  != -100 && new_scanner_period){
				  temp_scanner_period = scanner_command_msg.Scanner_Period;
				  ROS_INFO("Start Set Scanner Period Process");
    				  while (state != ESSIdle)
     			 	  {
       					cout << '.' << flush;
       					Time::sleep(1, 0);
       					retCode = rover.getScannerState(state);
     				  }
     			 	  retCode = rover.setScannerPeriod(temp_scanner_period);
      			 	  if (retCode != 0) ROS_ERROR("Set Scanner Home Angle failed");
				  else ROS_INFO("Scanner Period is successfully set to %f",temp_scanner_period);	
			}
		}

		void ScannerCommander(const geometry_msgs::Vector3::ConstPtr& vector)
		{
 			 geometry_msgs::Vector3 new_correction = *vector;
  			 float msg = new_correction.z;
  			 int retCode;
			 /*
			 retCode = rover.setScannerHomePosition(0);
  			 if (retCode != 0)
    			 {
     			          ROS_ERROR("Failed TO Set Scanner Home Position");
  			 }*/
 			 if (msg ==1 ){
    				  while (state != ESSIdle)
     			 	  {
       					cout << '.' << flush;
       					Time::sleep(1, 0);
       					retCode = rover.getScannerState(state);
     				  }
     			 	  retCode = rover.sendScannerCommand(ESCDoHoming);
      			 	  if (retCode != 0)
      		 	  	  {
        		  	       printError("GoHome failed", retCode);

      				  }
  			 } else if(msg == 2)
  			 {
     			 	while (state != ESSIdle)
      				{
       					cout << '.' << flush;
       					Time::sleep(1, 0);
       					retCode = rover.getScannerState(state);
      			 	}
      			 	retCode = rover.sendScannerCommand(ESCStart);
      			 	if (retCode != 0)
      			 	{
       		         		printError("Starting failed", retCode);

     				 }
  			} else if(msg == 3)
  			{
      				retCode = rover.sendScannerCommand(ESCStop);
      				if (retCode != 0)
      				{
       					printError("Stopping failed", retCode);

      				}
  			} else if (msg == 4)
  			{
     				retCode = rover.sendScannerCommand(ESCDoHoming);

  			}else
  			{
      			//printf(" \n Command is not recognized, recognized commands are GoHome, Start and Stop : \n");

  			}

		}


		void CMDVELLCommander(const geometry_msgs::Twist::ConstPtr& vel)
		{
  			geometry_msgs::Twist new_vel = *vel;
  			float v = sqrt (new_vel.linear.x * new_vel.linear.x + new_vel.linear.y * new_vel.linear.y);
  			float vth = new_vel.angular.z;
  			int retCode = rover.setSpeedVO (v, vth);
    			if (retCode != 0)
       			 {
        		// Failed to set the speed
        		ROS_ERROR("Failed to set speed of the rover");
 		 	}

		}

		void joyCallback(const sensor_msgs::JoyConstPtr& joy)
		{
  			float v=joy->axes[1];
  			float vth=joy->axes[2];
 		        float a1=joy->buttons[4];
  			v= v*(a1+1)/2;
  			int retCode = rover.setSpeedVO (v, vth);
    			if (retCode != 0)
			{
     				// Failed to set the speed
				ROS_ERROR("Failed to set speed of the rover");
  			}
  
		}

		// Class Functions
		void Scanner_Handle()
		{
			float syncf = 100.0;
    			EScannerState state;
    			int retCode = rover.setScannerPeriod(2.00);
    			retCode = rover.setScannerAdjustment(0.00000000);
    			retCode = rover.getScannerState(state);
    			while (state != ESSIdle)
    				{
        			cout << '.' << flush;
        			Time::sleep(1, 0);
        			retCode = rover.getScannerState(state);
   			 }

  			retCode = rover.setSyncFreq(syncf);
  			if (retCode != 0)
        			{
        			// Failed to set sync frequency
        			printf("Failed to set sync frequency to %f",syncf);
  			}
  			retCode = rover.setScannerPeriod(2);
  			if (retCode != 0)
  			{
      				ROS_ERROR("setScannerPeriod failed");
  			}

		}

		void Rover_Handle()
		{
  			int retCode = rover.init();
  			if (retCode != 0)
			{
				ROS_ERROR("Failed Initialize the Rover");
  			}
		}
		
		/*void initParams()
		{
		std::string send_odom;
   		n.param<std::string>("send_odom", send_odom, "false");
   		if (send_odom == "true") {send_odom_=true; ROS_INFO("Send odom true");}
   		else {send_odom_=false;ROS_INFO("Send odom false");}
    		//if (n_.getparam)

		}*/
		
		void RoverDataProvider()
		{
			Time timestamp;
			float temp_Front_Left_Track_Speed;
			float temp_Front_Right_Track_Speed;
			float temp_Rear_Left_Track_Speed;
			float temp_Rear_Right_Track_Speed;

			float temp_Front_Left_Track_Angle;
			float temp_Front_Right_Track_Angle;
			float temp_Rear_Left_Track_Angle;
			float temp_Rear_Right_Track_Angle;
			float temp_Rear_Bogie_Angle;

			float temp_Scanner_Period;
			float temp_Scanner_adjustment_angle;
			float temp_Scanner_roll_angle;
			
			float temp_Battery_Voltage;
			float temp_Front_Right_Track_Current;
			float temp_Front_Left_Track_Current;
			float temp_Rear_Right_Track_Current;
			float temp_Rear_Left_Track_Current;


			EScannerState state;
			
			int retCode; 
  			ros::Time current_time;
  			current_time = ros::Time::now();
			retCode = rover.setAngleReference(EDIFrontRightTrack,1.34);
			retCode = rover.setAngleReference(EDIFrontLeftTrack,-1.34);
			retCode = rover.setAngleReference(EDIRearRightTrack,1.34);
			retCode = rover.setAngleReference(EDIRearLeftTrack,-1.34);
				
			retCode = rover.getSpeedInMPerS(timestamp,
				  temp_Front_Left_Track_Speed, temp_Front_Right_Track_Speed,
				  temp_Rear_Left_Track_Speed,  temp_Rear_Right_Track_Speed  );
  			if (retCode != 0)
			{
				ROS_ERROR("Failed to Read Speed of Tracks, Try to Restart the Rover");
  			}
			retCode = rover.getAngles(timestamp,
				  temp_Front_Left_Track_Angle, temp_Front_Right_Track_Angle,
				  temp_Rear_Left_Track_Angle,  temp_Rear_Right_Track_Angle,
				  temp_Rear_Bogie_Angle);
  			if (retCode != 0)
			{
				ROS_ERROR("Failed to Read Angle of Tracks, Try to Restart the Rover");
  			}
			////////////////SCANNER STATE////////////////////	
			retCode = rover.getScannerState(state);
			if (retCode != 0)
			{
				ROS_ERROR("Failed to Read Scanner State, Try to Restart the Rover");
  			}
			if(state == ESSUnknown) 	 outputScanner.Scanner_State = "Unknown!";
			if(state == ESSIdle)   		 outputScanner.Scanner_State = "Idle";
			if(state == ESSCommandReceived ) outputScanner.Scanner_State = "CommandReceived";
			if(state == ESSHoming) 		 outputScanner.Scanner_State = "Homing";
			if(state == ESSGoingHome) 	 outputScanner.Scanner_State = "GoingHome";
			if(state == ESSRolling) 	 outputScanner.Scanner_State = "Rolling ";

			retCode = rover.getScannerPeriod(temp_Scanner_Period);
			if (retCode != 0)
			{
				ROS_ERROR("Failed to Read Scanner Period, Try to Restart the Rover");
  			}
			retCode = rover.getScannerAdjustment(temp_Scanner_adjustment_angle);
			if (retCode != 0)
			{
				ROS_ERROR("Failed to Read Scanner Adjustment angle, Try to Restart the Rover");
  			}
			retCode = rover.getScannerAngle(temp_Scanner_roll_angle);
			if (retCode != 0)
			{
				ROS_ERROR("Failed to Read Scanner Roll angle, Try to Restart the Rover");
  			}


			retCode = rover.readVoltage(EDIFrontRightTrack,timestamp,temp_Battery_Voltage);
			if (retCode != 0)
			{
				ROS_ERROR("Failed to Read Battery Voltage, Try to Restart the Rover");
  			}
			retCode = rover.getCurrent(EDIFrontRightTrack,timestamp,temp_Front_Right_Track_Current);
			if (retCode != 0)
			{
				ROS_ERROR("Failed to Read Battery Voltage, Try to Restart the Rover");
  			}
			retCode = rover.getCurrent(EDIFrontRightTrack,timestamp,temp_Front_Right_Track_Current);
			if (retCode != 0)
			{
				ROS_ERROR("Failed to Read Front_Right_Track_Current, Try to Restart the Rover");
  			}
			retCode = rover.getCurrent(EDIFrontLeftTrack,timestamp,temp_Front_Left_Track_Current);
			if (retCode != 0)
			{
				ROS_ERROR("Failed to Read Front_Left_Track_Current, Try to Restart the Rover");
  			}
			retCode = rover.getCurrent(EDIRearRightTrack,timestamp,temp_Rear_Right_Track_Current);
			if (retCode != 0)
			{
				ROS_ERROR("Failed to Read Rear_Right_Track_Current, Try to Restart the Rover");
  			}
			retCode = rover.getCurrent(EDIRearLeftTrack,timestamp,temp_Rear_Left_Track_Current);
			if (retCode != 0)
			{
				ROS_ERROR("Failed to Read Rear_Left_Track_Current, Try to Restart the Rover");
  			}			


			outputTrackSpeed.Front_Left_Track_Speed	  = temp_Front_Left_Track_Speed;
			outputTrackSpeed.Front_Right_Track_Speed  = temp_Front_Right_Track_Speed;
			outputTrackSpeed.Rear_Left_Track_Speed    = temp_Rear_Left_Track_Speed;
			outputTrackSpeed.Rear_Right_Track_Speed   = temp_Rear_Right_Track_Speed;
			outputTrackSpeed.header.stamp = current_time;
   	 		outputTrackSpeed.header.frame_id = "base_link";
			outputTrackSpeed.TimeStamp = current_time.toSec();

			outputBogieAngle.Front_Left_Track_Angle   = temp_Front_Left_Track_Angle;
			outputBogieAngle.Front_Right_Track_Angle  = temp_Front_Right_Track_Angle;
			outputBogieAngle.Rear_Left_Track_Angle    = temp_Rear_Left_Track_Angle;
			outputBogieAngle.Rear_Right_Track_Angle   = temp_Rear_Right_Track_Angle;
			outputBogieAngle.Rear_Bogie_Angle         = temp_Rear_Bogie_Angle -1.31;
 			outputBogieAngle.header.stamp = current_time;
   	 		outputBogieAngle.header.frame_id = "base_link";
			outputBogieAngle.TimeStamp = current_time.toSec();
			
			outputScanner.Scanner_Period = temp_Scanner_Period;
			outputScanner.Scanner_adjustment_angle = temp_Scanner_adjustment_angle;
			outputScanner.Scanner_roll_angle = temp_Scanner_roll_angle;
			outputScanner.Scanner_angle = scannerCal;
			outputScanner.Scanner_angle_encoder = scannerCal;
			outputScanner.Scanner_angle_degree = (scannerCal/21687)*90;
			outputScanner.Scanner_angle = (scannerCal/21687)*M_PI/2;
 			outputScanner.header.stamp = current_time;
   	 		outputScanner.header.frame_id = "base_link";
			outputScanner.TimeStamp = current_time.toSec();

			outputPower.Battery_Voltage = temp_Battery_Voltage;
			outputPower.Front_Right_Track_Current = temp_Front_Right_Track_Current;
			outputPower.Front_Left_Track_Current  = temp_Front_Left_Track_Current;
			outputPower.Rear_Right_Track_Current  = temp_Rear_Right_Track_Current;
			outputPower.Rear_Left_Track_Current   = temp_Rear_Left_Track_Current;
 			outputPower.header.stamp = current_time;
   	 		outputPower.header.frame_id = "base_link";
			outputPower.TimeStamp = current_time.toSec();
			if(temp_Battery_Voltage < 46.00) Low_Battery = true;
			else Low_Battery = false;

      
			Rover_Track_Speed_pub.publish(outputTrackSpeed); 
			Rover_Track_Angles_pub.publish(outputBogieAngle);
			Rover_Scanner_Data_pub.publish(outputScanner);
			Rover_Power_Data_pub.publish(outputPower);

		}

                void Odometry_Handle()
		{
		        // Handling the parameters
  			ros::NodeHandle n("~");
   			n.param("send_odom", send_odom_, false);
   			
   			//Variables
  			float x = 0.0;
  			float y = 0.0;
  			float th = 0.0;
  			float v;
  			float vx = 0.1;
  			float vy = -0.1;
 		 	float vth;
  			int count = 0;
  			cout << '*' <<endl;
  			ROS_INFO("Sending Odom Transform by rover_odom : %d", send_odom_);
  			if (Low_Battery) ROS_WARN("Battery Low");
			// Start Scanner angle calcultion - Defining variables
			float delta_scanner = 0;
			float last_scanner_value = 0;
			float scanner_offsetVal = 0;
			tf::TransformBroadcaster broadcaster1;
            tf::TransformBroadcaster broadcaster2;
			// End Scanner angle calcultion - Defining variables
			int retCode;
  			ros::Time current_time, last_time;
  			current_time = ros::Time::now();
  			last_time = ros::Time::now();
			ros::Rate loop_rate(rate);
  			while(ros::ok()){

				// Start Scanner Angle Loop 
    				delta_scanner =scannerRaw - last_scanner_value;
				if (delta_scanner > 10000)       scanner_offsetVal = scanner_offsetVal - scannerRaw;
				else if(delta_scanner < -10000)  scanner_offsetVal = scanner_offsetVal + last_scanner_value;
				last_scanner_value = scannerRaw;
				scannerCal = scannerRaw + scanner_offsetVal;
    				tf::Quaternion scan_quart,scan_quart_back;
    				scan_quart.setRPY(((scannerCal/21687)*M_PI/2),0.0 , 0.0);
                    scan_quart_back.setRPY((-(scannerCal/21687)*M_PI/2),0.0 , 0.0);
    				broadcaster1.sendTransform(
      				tf::StampedTransform(

        			tf::Transform(scan_quart, tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(),"laser","base_laser")
    				);
    				broadcaster2.sendTransform(
      				tf::StampedTransform(

        			tf::Transform(scan_quart_back, tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(),"base_laser","base_scanner")
    				);
				// END Scanner Angle Loop
    				Time timestamp;
    				ros::spinOnce();               // check for incoming messages
    				current_time = ros::Time::now();
    				// Reading speed of the rover
    				/*retCode = rover.getSpeedInMPerS(timestamp, v_left, v_right, v_left2, v_right2);

    				if (retCode != 0)
        			{
        				// Failed to read the rover speed
        				ROS_ERROR("Failed to read the Righ_left speed");
    				}*/

    				retCode = rover.getSpeedVO(timestamp, v, vth);
    				if (retCode != 0)
				{
     					// Failed to read the rover speed
					ROS_ERROR("Failed to read the rover speed");
   				 }

    				/*if ((count>50) && ((v > 0.01) || (vth > 0.01) || (vth < -0.01)))
        				{
        				printf("Front left speed is   %f   while rear left speed is   %f   \n", v_left, v_left2);
        				printf("Front right speed is   %f   while rear right speed is   %f   \n .. \n .. \n .. \n", v_right, v_right2);

    				}*/

    				// Correction applied
    				vx = v * cos(th);    
    				vy = v * sin(th);



    				//compute odometry in a typical way given the velocities of the robot
    				float dt = (current_time - last_time).toSec();
    				float delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    				float delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    				float delta_th = vth * dt;
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
    				odom_trans.child_frame_id = "base_link";

    				odom_trans.transform.translation.x = x;
   				odom_trans.transform.translation.y = y;
    				odom_trans.transform.translation.z = 0.0;
    				odom_trans.transform.rotation = odom_quat;

    				//send the transform
    				if (send_odom_ == true) odom_broadcaster.sendTransform(odom_trans);
    				//odom_broadcaster.sendTransform(odom_trans);

    				//next, we'll publish the odometry message over ROS
    				
    				odom.header.stamp = current_time;
   	 			odom.header.frame_id = "odom";
    				

    				//set the position
    				odom.pose.pose.position.x = x;
    				odom.pose.pose.position.y = y;
    				odom.pose.pose.position.z = 0.0;
    				odom.pose.pose.orientation = odom_quat;

    				//set the velocity
    				odom.child_frame_id = "base_link";
    				odom.twist.twist.linear.x = vx;
    				odom.twist.twist.linear.y = vy;
    				odom.twist.twist.angular.z = vth;
    				tw.linear.x = VX;
    				tw.linear.y = VY;

    				//publish the message
    				odom_pub.publish(odom);
    				twist_pub.publish(tw);
				RoverDataProvider();
    				last_time = current_time;
    				count ++;
				loop_rate.sleep();
  				}
		}
				
		

	protected:
		/*state here*/
		ros::NodeHandle n_;
		
		// Subscribers
		ros::Subscriber subFromJoystick_;
		ros::Subscriber subFromCMDVEL_;
		ros::Subscriber subFromScannerCommander_;
		//ros::Subscriber subFromIMUSpeed_;
	        ros::Subscriber subFromRightLeftCommands_;
		ros::Subscriber subFromscannerdata_;
		ros::Subscriber subFromscannercommands_;
		// Publishers
		ros::Publisher odom_pub;
		ros::Publisher twist_pub;
		ros::Publisher Rover_Track_Speed_pub;
		ros::Publisher Rover_Track_Angles_pub;
		ros::Publisher Rover_Scanner_Data_pub;
		ros::Publisher Rover_Power_Data_pub;

		nav_msgs::Odometry odom;
		tf::TransformBroadcaster odom_broadcaster;
		geometry_msgs::Twist tw;
		donkey_rover::Rover_Track_Speed outputTrackSpeed;
		donkey_rover::Rover_Scanner outputScanner;
		donkey_rover::Rover_Track_Bogie_Angle outputBogieAngle;
		donkey_rover::Rover_Power_Data outputPower;

		int rate = 100;
		bool send_odom_;

	private:

		float temp_adjustment_angle = -100;
        float temp_roll_angle = -100;
        float temp_home_angle = -100;
        float temp_scanner_period = -100;
        string temp_scanner_command = "Ciao";
		
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "donkey_rover");
	ros::NodeHandle node;

	DonkeyRoverClass DonkeyRoverNode(node);

	DonkeyRoverNode.Rover_Handle();
	
	DonkeyRoverNode.Scanner_Handle();
	//DonkeyRoverNode.RoverDataProvider();
	DonkeyRoverNode.Odometry_Handle();
	return 0;
}
