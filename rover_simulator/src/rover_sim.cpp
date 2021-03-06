#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
//#include "libRover.h"
#include <sensor_msgs/Joy.h>
#include <math.h>
#include "donkey_rover/Scanner_Command.h"
#include "donkey_rover/Rover_Track_Speed.h"
#include "donkey_rover/Rover_Track_Bogie_Angle.h"
#include "donkey_rover/Rover_Scanner.h"
#include "donkey_rover/Rover_Power_Data.h" 
#include <math.h>
#include <custom_msgs/gnssSample.h>
#include <sensor_msgs/Imu.h>

//Rover rover(false);
//EScannerState state;
float scannerRaw = 0;
float scannerCal = 0;
float VX = 0.0;
float VY = 0.0;
bool Low_Battery = false;
float radius = 6378137; // [m] earth radius 

class DonkeyRoverClass
{
	public:
		
		DonkeyRoverClass(ros::NodeHandle& node)
		{
			n_=node;

			//subscribers
			subFromJoystick_ 		= n_.subscribe("joy", 1, &DonkeyRoverClass::joyCallback,this);
			subFromCMDVEL_ 			= n_.subscribe("cmd_vel", 1, &DonkeyRoverClass::CMDVELLCommander,this);
			//subFromScannerCommander_ 	= n_.subscribe("c1", 10, &DonkeyRoverClass::ScannerCommander,this);
			//subFromIMUSpeed_= n_.subscribe("mti/filter/velocity", 10, &DonkeyRoverClass::imuspeed,this);
			subFromRightLeftCommands_	= n_.subscribe("speedfollow", 5, &DonkeyRoverClass::RLcommander,this);
			//subFromscannerdata_		= n_.subscribe("scanner_data", 1, &DonkeyRoverClass::scannerRawValueSet,this);
			//subFromscannercommands_		= n_.subscribe("scanner_commands", 1, &DonkeyRoverClass::SetScanner,this);

			// publishers
			odom_pub 	   	  = n_.advertise<nav_msgs::Odometry>("odom", 100);
			twist_pub 	  	  = n_.advertise<geometry_msgs::Twist>("twist", 100);
			Rover_Track_Speed_pub     = n_.advertise<donkey_rover::Rover_Track_Speed>("RoverTrackSpeed", 100);
			Rover_Track_Angles_pub    = n_.advertise<donkey_rover::Rover_Track_Bogie_Angle>("RoverTrackAngles", 100);	
			Rover_Scanner_Data_pub    = n_.advertise<donkey_rover::Rover_Scanner>("RoverScannerInfo", 100);
			Rover_Power_Data_pub      = n_.advertise<donkey_rover::Rover_Power_Data>("RoverPowerInfo", 100);
			GPS_SIM_pub		  = n_.advertise<custom_msgs::gnssSample>("/mti/sensor/gnssPvt", 100);
            		IMU_SIM_pub		  = n_.advertise<sensor_msgs::Imu>("/mti/sensor/imu", 100);
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
  			VL = speed.y;
  			VR = speed.x;
			v = (VL+VR)/2;
			vth = (VR-VL)/R;
		}
		
		/*void scannerRawValueSet(const geometry_msgs::Vector3::ConstPtr& vector)
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
		
		if (temp_adjustment_angle == scanner_command_msg.Scanner_Ajustment_Angle) new_adjustment_angle=false;
		else new_adjustment_angle=true;
		if (temp_roll_angle == scanner_command_msg.Scanner_Roll_Angle) new_roll_angle=false;
		else new_roll_angle=true;
		if (temp_home_angle == scanner_command_msg.Scanner_Home_Angle) new_home_angle=false;
		else new_home_angle=true;
		if (temp_scanner_period == scanner_command_msg.Scanner_Period) new_scanner_period=false;
		else new_scanner_period=true;
		if (temp_scanner_command == scanner_command_msg.Scanner_Command) new_scanner_command=false;
		else new_scanner_command=true;

		float temp_roll_angle;
		float temp_home_angle = -100;
		float temp_scanner_period = -100;
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

  			}else
				ROS_ERROR("Scanner Command is not valid"); 
  			{
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
			if (scanner_command_msg.Scanner_Home_Angle != -100  && new_home_angle){
				  temp_home_angle = scanner_command_msg.Scanner_Home_Angle;
				  ROS_INFO("Start Set Scanner Home Angle Process");
    				  while (state != ESSIdle)
     			 	  {
       					cout << '.' << flush;
       					Time::sleep(1, 0);
       					retCode = rover.getScannerState(state);
     				  }
     			 	  retCode = rover.setScannerHomePosition(temp_home_angle);
      			 	  if (retCode != 0) ROS_ERROR("Set Scanner Home Angle failed");
				  else ROS_INFO("Scanner Home Angle is successfully set to %f",temp_home_angle);	
			}
			//Scanner Period 
			if (scanner_command_msg.Scanner_Period != -100 && new_scanner_period){
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
		}}*/

		/*void ScannerCommander(const geometry_msgs::Vector3::ConstPtr& vector)
		{
 			 geometry_msgs::Vector3 new_correction = *vector;
  			 float msg = new_correction.z;
  			 int retCode;

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

		}*/


		void CMDVELLCommander(const geometry_msgs::Twist::ConstPtr& vel)
		{
  			geometry_msgs::Twist new_vel = *vel;
  			v = sqrt (new_vel.linear.x * new_vel.linear.x + new_vel.linear.y * new_vel.linear.y);
  			vth = new_vel.angular.z;
			VL = (R*vth+2*v)/2;
			VR = 2*v -VL; 

		}

		void joyCallback(const sensor_msgs::JoyConstPtr& joy)
		{
  			v=joy->axes[1];
  			vth=joy->axes[2];
 		        float a1=joy->buttons[4];
  			v= v*(a1+1)/2;
			VL = (R*vth+2*v)/2;
			VR = 2*v -VL;   			
  
		}

		// Class Functions
		/*
		void Scanner_Handle()
		{
			float syncf = 100.0;
    			EScannerState state;
    			int retCode = rover.getScannerState(state);
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
		}*/
		
		void RoverDataProvider()
		{
			//Time timestamp;
			float temp_Front_Left_Track_Speed;
			float temp_Front_Right_Track_Speed;
			float temp_Rear_Left_Track_Speed;
			float temp_Rear_Right_Track_Speed;
			/*
			float temp_Front_Left_Track_Angle;
			float temp_Front_Right_Track_Angle;
			float temp_Rear_Left_Track_Angle;
			float temp_Rear_Right_Track_Angle;
			float temp_Rear_Bogie_Angle;

			float temp_Scanner_Period;
			float temp_Scanner_adjustment_angle;
			
			float temp_Battery_Voltage;
			float temp_Front_Right_Track_Current;
			float temp_Front_Left_Track_Current;
			float temp_Rear_Right_Track_Current;
			float temp_Rear_Left_Track_Current;
			

			EScannerState state;
			
			int retCode; */
  			ros::Time current_time;
  			current_time = ros::Time::now();
			temp_Front_Left_Track_Speed = VL;
			temp_Rear_Left_Track_Speed = VL;
			temp_Front_Right_Track_Speed = VR;
			temp_Rear_Right_Track_Speed = VR;
			/*
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
  			}*/			


			outputTrackSpeed.Front_Left_Track_Speed	  = temp_Front_Left_Track_Speed;
			outputTrackSpeed.Front_Right_Track_Speed  = temp_Front_Right_Track_Speed;
			outputTrackSpeed.Rear_Left_Track_Speed    = temp_Rear_Left_Track_Speed;
			outputTrackSpeed.Rear_Right_Track_Speed   = temp_Rear_Right_Track_Speed;
			outputTrackSpeed.header.stamp = current_time;
   	 		outputTrackSpeed.header.frame_id = "base_link";
			outputTrackSpeed.TimeStamp = current_time.toSec();

			outputBogieAngle.Front_Left_Track_Angle   = 0;
			outputBogieAngle.Front_Right_Track_Angle  = 0;
			outputBogieAngle.Rear_Left_Track_Angle    = 0;
			outputBogieAngle.Rear_Right_Track_Angle   = 0;
			outputBogieAngle.Rear_Bogie_Angle         = 0;
 			outputBogieAngle.header.stamp = current_time;
   	 		outputBogieAngle.header.frame_id = "base_link";
			outputBogieAngle.TimeStamp = current_time.toSec();
			
			outputScanner.Scanner_State = "Idle";
			outputScanner.Scanner_Period = 2;
			outputScanner.Scanner_adjustment_angle = 0;
			outputScanner.Scanner_angle = 0;
			outputScanner.Scanner_angle_encoder = 0;
			outputScanner.Scanner_angle_degree = 0;
			outputScanner.Scanner_angle = 0;
 			outputScanner.header.stamp = current_time;
   	 		outputScanner.header.frame_id = "base_link";
			outputScanner.TimeStamp = current_time.toSec();

			outputPower.Battery_Voltage = 54;
			outputPower.Front_Right_Track_Current = VR*0.3;
			outputPower.Front_Left_Track_Current  = VL*0.3;
			outputPower.Rear_Right_Track_Current  = VR*0.3;
			outputPower.Rear_Left_Track_Current   = VL*0.3;
 			outputPower.header.stamp = current_time;
   	 		outputPower.header.frame_id = "base_link";
			outputPower.TimeStamp = current_time.toSec();

      
			Rover_Track_Speed_pub.publish(outputTrackSpeed); 
			Rover_Track_Angles_pub.publish(outputBogieAngle);
			Rover_Scanner_Data_pub.publish(outputScanner);
			Rover_Power_Data_pub.publish(outputPower);

		}

		void GPS_From_XY()
		{
			//Calculate Lat and Long from global variables x and y
			gps.latitude =  14.141515 + x/radius*180.0/M_PI; // x = position = NORTH
			gps.longitude = 14.141515 - y/radius*180.0/M_PI;  //y = position = EAST
			gps.hEll = 0.00;//To be Done
			gps.hMsl= 0.00;//To be Done
			GPS_SIM_pub.publish(gps);
		
		}

        void Odometry_Handle()
		{
			R = 0.8;
			rate = 100;
  			x = 0.0;
  			y = 0.0;
  			th = -M_PI/4;
  			float vx = 0.1;
  			float vy = -0.1;
  			int count = 0;
			// Start Scanner angle calcultion - Defining variables
			float delta_scanner = 0;
			float last_scanner_value = 0;
			float scanner_offsetVal = 0;
			tf::TransformBroadcaster broadcaster;
			// End Scanner angle calcultion - Defining variables
			ROS_INFO_ONCE("Rover Simulator Started");
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
    				tf::Quaternion scan_quart;
    				scan_quart.setRPY(((scannerCal/21687)*M_PI/2),0.0 , 0.0);
    				broadcaster.sendTransform(
      				tf::StampedTransform(

        			tf::Transform(scan_quart, tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(),"base_laser","base_scanner")
    				);
				// END Scanner Angle Loop
    				//Time timestamp;
    				ros::spinOnce();               // check for incoming messages
    				current_time = ros::Time::now();
    				// Reading speed of the rover
    				/*retCode = rover.getSpeedInMPerS(timestamp, v_left, v_right, v_left2, v_right2);

    				if (retCode != 0)
        			{
        				// Failed to read the rover speed
        				ROS_ERROR("Failed to read the Righ_left speed");
    				}

    				retCode = rover.getSpeedVO(timestamp, v, vth);
    				if (retCode != 0)
				{
     					// Failed to read the rover speed
					ROS_ERROR("Failed to read the rover speed");
   				 }*/

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
    				odom_broadcaster.sendTransform(odom_trans);

    				//next, we'll publish the odometry message over ROS
    				//nav_msgs::Odometry odom;
    				odom.header.stamp = current_time;
   	 			    odom.header.frame_id = "odom";
    				//geometry_msgs::Twist tw;

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
                    
                    //Create the simulated Imu
                    sensor_msgs::Imu sim_imu;
                    geometry_msgs::Quaternion sim_quat;
                    sim_quat.w = cos(th/2);  //ENU standard
                    sim_quat.x = 0;
                    sim_quat.y = 0;
                    sim_quat.z = sin(th/2);
                    sim_imu.orientation = sim_quat;//odom_quat;

    				//publish the message
    				odom_pub.publish(odom);
    				twist_pub.publish(tw);
                    		IMU_SIM_pub.publish(sim_imu);
				    RoverDataProvider();
				    GPS_From_XY();
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
		//ros::Subscriber subFromscannerdata_;
		//ros::Subscriber subFromscannercommands_;
		// Publishers
		ros::Publisher odom_pub;
		ros::Publisher twist_pub;
		ros::Publisher Rover_Track_Speed_pub;
		ros::Publisher Rover_Track_Angles_pub;
		ros::Publisher Rover_Scanner_Data_pub;
		ros::Publisher Rover_Power_Data_pub;
		ros::Publisher GPS_SIM_pub;
		ros::Publisher IMU_SIM_pub;

		nav_msgs::Odometry odom;
		tf::TransformBroadcaster odom_broadcaster;
		geometry_msgs::Twist tw;
		donkey_rover::Rover_Track_Speed outputTrackSpeed;
		donkey_rover::Rover_Scanner outputScanner;
		donkey_rover::Rover_Track_Bogie_Angle outputBogieAngle;
		donkey_rover::Rover_Power_Data outputPower;
		custom_msgs::gnssSample gps;
		float v = 0;
		float vth = 0;
 		float VL = 0;
		float VR = 0;
		float R = 0.8;
  		float x;
  		float y;
  		float th;
		int rate = 100; 

	private:
		//float temp_adjustment_angle = -100;
		//float temp_roll_angle = -100;
		//float temp_home_angle = -100;
		//float temp_scanner_period = -100;
		
		
		
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "donkey_rover");
	ros::NodeHandle node;

	DonkeyRoverClass DonkeyRoverNode(node);

	//DonkeyRoverNode.Rover_Handle();
	
	//DonkeyRoverNode.Scanner_Handle();
	//DonkeyRoverNode.RoverDataProvider();
	DonkeyRoverNode.Odometry_Handle();
	return 0;
}
