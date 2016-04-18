#include "ros/ros.h"

#include "sherpa_msgs/Cmd.h"
#include "sherpa_msgs/MMS_status.h"
#include "sherpa_msgs/Distance.h"	
#include "sherpa_msgs/LeashingCommand.h"   //leashing
#include "sherpa_msgs/LeashingStatus.h"    //leashing
#include "sherpa_msgs/GeoPoint.h"	 //leashing
#include "geometry_msgs/Vector3.h" 
#include "custom_msgs/gnssSample.h"
#include <wgs84_ned_lib/wgs84_ned_lib.h>       

//double M_PI = 3.1416; // pi
bool is_pos = false;
bool is_outputRef_ready = false;
bool is_GPS_present = false;
bool is_new_wp = false;
bool is_new_leashing = false;
bool home_set = false;
bool is_paused = false;

class ReferenceNodeClass
{
	public:
	ReferenceNodeClass(ros::NodeHandle& node)
	{
		n_=node;

		//subscribers
		subFromPosition_ = n_.subscribe("/estimated_position", 10, &ReferenceNodeClass::readPositionMessage,this);
		subFromCmd_ = n_.subscribe("/cmd_from_mms", 10, &ReferenceNodeClass::readCmdMessage,this);     //filtered by mms
		subFromMmsStatus_ = n_.subscribe("/mms_status", 10, &ReferenceNodeClass::readMmsStatusMessage,this);
		subLeashingTargetPosition_ = n_.subscribe("/leashing_target_position", 10, &ReferenceNodeClass::readLeashingTarget,this);
		subFromGPS_ = n_.subscribe("/mti/sensor/gnssPvt", 10, &ReferenceNodeClass::readGPSMessage,this);	
		
		// publishers
		pubToReference_    = n_.advertise<sherpa_msgs::GeoPoint>("/reference",10);
		pubToDistance_     = n_.advertise<sherpa_msgs::Distance>("/distance",10);
		pubLeashingStatus_ = n_.advertise<sherpa_msgs::LeashingStatus>("/leashing_status",5);
		pubHome_           = n_.advertise<sherpa_msgs::GeoPoint>("/home",5);

		// STATE INITIALIZATION
		currentState = NO_HOME;
		oldState = 0;//NO_HOME;

		// rate = 10;
		new_state = true;
		
		// counter_print = 0;

		leashing_status_.failure = 0;
		// tracking_error = 0;
	}

	void readGPSMessage(const custom_msgs::gnssSample::ConstPtr& msg) 				
		{	
			/*  # This is a message to hold data a GNSS unit
				# Supported for MTi Devices with FW 1.4 and above.

				Header header

				float64 itow
				float64 fix

				float64 latitude
				float64 longitude
				float64 hEll
				float64 hMsl

				# ENU velocity
				geometry_msgs/Vector3 vel // float64

				float64 hAcc
				float64 vAcc
				float64 sAcc

				float64 pDop
				float64 hDop
				float64 vDop

				float64 numSat
				float64 heading
				float64 headingAcc
			*/
			inputGPS_.heading = msg->heading; 
			inputGPS_.hMsl = msg->hMsl; 
			inputGPS_.hEll = msg->hEll; 
			inputGPS_.latitude = msg->latitude;
			inputGPS_.longitude = msg->longitude;
			inputGPS_.vel.x = msg->vel.x; //East
			inputGPS_.vel.y = msg->vel.y; //North
			inputGPS_.vel.z = msg->vel.z; //Up
		
			//ROS_INFO("NEW GPS MSG, LAT, LON %.7f, %.7f",inputGPS_.latitude, inputGPS_.longitude); // ONLY FOR DEBUG
		
			//is_GPS_present = true;
			//GPS_msg_count = GPS_msg_count+1;
			//ROS_INFO("GPS_msg_count %.0f",GPS_msg_count); // ONLY FOR DEBUG
			/*if (GPS_msg_count == 1)
			{
				GPS_first_msg = true;
			}
			else if (GPS_msg_count > 1)
			{
				GPS_msg_count = 2;
				GPS_first_msg = false;
			}*/
			ROS_INFO_ONCE("REF: GPS_ RECEIVED");
			is_GPS_present = true;
			Reference_Handle();
		}
	
	void readMmsStatusMessage(const sherpa_msgs::MMS_status::ConstPtr& msg)
	{
		inputMmsStatus_.mms_state=msg->mms_state;
		// inputMmsStatus_.target_ref_frame=msg->target_ref_frame;
		//ROS_INFO("REF: MMS_status received %d", inputMmsStatus_.mms_state);
		currentState = inputMmsStatus_.mms_state;
		//new_state = true;
		ROS_INFO("REF: MMS STATUS RECEIVED: %d", currentState);
		Reference_Handle();
	}
	
	void distance()
	{
		double error_x, error_y;

		get_pos_NED_from_WGS84(&error_x, &error_y, current_rover_pos_.latitude, current_rover_pos_.longitude, outputRef_.latitude, outputRef_.longitude);

		outputDist_.error_pos = sqrt(error_x*error_x + error_y*error_y);  //m;
		outputDist_.seq = inputCmd_.seq;
		/*counter_print++;
		if (counter_print >= 30)
		{
			counter_print = 0;
			ROS_INFO("DISTANCE TO TARGET: Linear [mm] %f", outputDist_.error_pos);
		}*/
		pubToDistance_.publish(outputDist_);
	}

	void set_current_position_as_ref()
	{
		outputRef_ = current_rover_pos_;
		// ROS_INFO("CURRENT: lat, lon %f , %f", current_rover_pos_.latitude, current_rover_pos_.longitude);
	}

	void readLeashingTarget(const sherpa_msgs::GeoPoint::ConstPtr& msg)
	{
		leashing_target_ = *msg;
		//ROS_INFO_ONCE("REF: LEASHING TARGET RECEIVED");
		is_new_leashing = true;
		Reference_Handle();
		/*if ((leashing_target_.latitude != 0.0) && (leashing_target_.longitude != 0.0)) 
		{	
			if ((leashing_target_.latitude < 90.0) && (leashing_target_.latitude > -90.0) && (leashing_target_.longitude < 180.0) && (leashing_target_.longitude > -180.0))
			{
				ROS_INFO("LEASHING TARGET VALID");
				is_new_leashing = true;
				Reference_Handle();
			}
		}
		else
		{
			ROS_INFO("LEASHING TARGET NOT VALID");
		}*/
	}
	
	void readPositionMessage(const sherpa_msgs::GeoPoint::ConstPtr& msg)
	{
		current_rover_pos_ = *msg;
		is_pos = true;
		// ROS_INFO("ROVER ESTIMATED POSITION RECEIVED");
		Reference_Handle();
	}	

	void readCmdMessage(const sherpa_msgs::Cmd::ConstPtr& msg)
	{
		ROS_INFO("REF: CMD_RECEIVED");
		inputCmd_.command = msg -> command;
		inputCmd_.param1  = msg -> param1;
		inputCmd_.param2  = msg -> param2;
		inputCmd_.param3  = msg -> param3;
		inputCmd_.param4  = msg -> param4;
		inputCmd_.param5  = msg -> param5;
		inputCmd_.param6  = msg -> param6;
		inputCmd_.param7  = msg -> param7;
		// inputCmd_.frame  = msg -> frame;
		inputCmd_.seq  = msg -> seq;

		switch(inputCmd_.command)
		{
			case 16:  // MAV_CMD_NAV_WAYPOINT
			{
				ROS_INFO("REF: MAV_CMD_DO_NAV_WAYPOINT");
				is_new_wp = true;
				outputDist_.command = 16;
			} break;

			case 179: // MAV_CMD_DO_SET_HOME
			{
				ROS_INFO("REF: MAV_CMD_DO_SET_HOME");
				home_set = false;
				outputDist_.command = 179;
			}break;

			case 25: // MAV_CMD_NAV_FOLLOW (LEASHING)
			{
				if (inputCmd_.param1 == 1)
				{
					leashing_status_.failure = 0;
				}
				outputDist_.command = 25;
			}break;
			
			case 252:  // MAV_CMD_OVERRIDE_GOTO (PAUSE/CONTINUE)
			{
				//seq_number = inputCmd_.seq;
				if (inputCmd_.param1 == 0)
				{          //PAUSE
					ROS_INFO("REF: CMD_PAUSE");
					//pubCmd_.publish(inputCmd_);  //cmd passed to reference
				} else if (inputCmd_.param1 == 1)
				{		//CONTINUE
					ROS_INFO("REF: CMD_CONTINUE");
					//pubCmd_.publish(inputCmd_);  //cmd passed to reference
				}
				outputDist_.command = 252;				
			}break;
		}
		Reference_Handle();
	}

	void Reference_Handle()
	{
		/*if (currentState != oldState)
		{
			new_state = true;
			oldState = currentState;
			ROS_INFO("REF: NEW MMS STATE");
		}*/
		new_state = true;
		
		switch(currentState)
		{
			/*case NO_HOME:
			{
				ROS_INFO("MMS CURRENT STATE: NO_HOME");
				if (new_state && is_pos)
				{
					ROS_INFO("REF: NO_HOME");
					new_state = false;
					set_current_position_as_ref();
					is_outputRef_ready = true;
					ROS_INFO("REFERENCE: lat, lon, alt %f , %f, %f", outputRef_.latitude, outputRef_.longitude, outputRef_.altitude);	
					is_pos = false;					
				}
			}break;*/

			case SETTING_HOME:
			{	
				// ROS_INFO("MMS CURRENT STATE: SETTING_HOME");
				if (new_state && is_GPS_present && !home_set)
				{
					new_state = false;
					ROS_INFO("REF: SETTING_HOME");
					Home_.latitude = inputGPS_.latitude;
					Home_.longitude = inputGPS_.longitude;
					Home_.altitude = inputGPS_.hEll;
					pubHome_.publish(Home_);
					is_GPS_present = false;
					home_set = true;
					is_paused = true;
				}
			}break;

			case READY_TO_GO:
			{
				// ROS_INFO("MMS CURRENT STATE: READY_TO_GO");
				if (new_state && is_pos && !is_paused)
				{
					ROS_INFO("REF: CURRENT POSITION");
					new_state = false;
					//set_current_position_as_ref();
					is_outputRef_ready = true;
					is_paused = true;
				}
			}break;			

			case PERFORMING_GO_TO:
			{	
				// ROS_INFO("MMS CURRENT STATE: PERFORMING_GO_TO");
				if (new_state && is_new_wp)
				{
					ROS_INFO("REF: PERFORMING_GO_TO");
					new_state = false;
					is_new_wp = false;					
					outputRef_.latitude = inputCmd_.param5; // TODO CHECK THE CONVERSION
					outputRef_.longitude = inputCmd_.param6; // TODO CHECK THE CONVERSION
					outputRef_.altitude = inputCmd_.param7; // TODO CHECK THE CONVERSION
					is_outputRef_ready = true;
					is_paused = false;
				}
			}break;

			case LEASHING:
			{
				// ROS_INFO("MMS CURRENT STATE: LEASHING");
				if (new_state && is_new_leashing)
				{
					ROS_INFO("REF: LEASHING");
					new_state = false;
					outputRef_ = leashing_target_; 
					is_outputRef_ready = true;
					is_new_leashing = false;
					pubLeashingStatus_.publish(leashing_status_);  //TODO maybe modify this to include failures
					is_paused= false;
				}
				
			}break;
		
			case PAUSED:
			{
				// ROS_INFO("MMS CURRENT STATE: PAUSED");
				if (new_state && is_pos && !is_paused)
				{
					ROS_INFO("REF: PAUSED");
					new_state = false;
					set_current_position_as_ref();
					is_outputRef_ready = true;
					is_paused = true;
				}
			}break;
		}
		if(is_outputRef_ready)
		{
			pubToReference_.publish(outputRef_);
			is_outputRef_ready = false;
			if (is_pos)
			{
				distance();		//calculate distance to target
			}
		}
	}

	void run()
	{
		ROS_INFO_ONCE("REF: RUNNING");
		ros::spin();
		/*ros::Rate loop_rate(rate);
		while (ros::ok())
		{
			ROS_INFO_ONCE("REF: RUNNING");

			Reference_Handle();
			ros::spinOnce();

			loop_rate.sleep();
		}*/
	}

	protected:
	/*state here*/
	ros::NodeHandle n_;

	ros::Subscriber subFromPosition_;
	ros::Subscriber subFromCmd_;
	ros::Subscriber subFromMmsStatus_;
	ros::Subscriber subFromGlobPosInt_;
	ros::Subscriber subLeashingTargetPosition_;
	ros::Subscriber subLeashingCommand_;

	ros::Publisher pubToReference_;
	ros::Publisher pubToDistance_;
	
	ros::Publisher pubLeashingStatus_;
	// ros::Publisher pubTrackingError_;

	ros::Subscriber subFromGPS_;
	custom_msgs::gnssSample inputGPS_;	

	ros::Publisher pubHome_;
    sherpa_msgs::GeoPoint Home_;	
	
	sherpa_msgs::Cmd inputCmd_;
	sherpa_msgs::MMS_status inputMmsStatus_;

	sherpa_msgs::GeoPoint outputRef_;

	sherpa_msgs::Distance outputDist_;

	sherpa_msgs::GeoPoint current_rover_pos_;

	sherpa_msgs::LeashingCommand leashing_command_;   //leashing
	sherpa_msgs::LeashingStatus leashing_status_;    //leashing
	sherpa_msgs::GeoPoint leashing_target_;	 //leashing
	

	// STATES DEFINITION
	static const int NO_HOME = 10;         //TODO make a .h to include in both reference and mms
	static const int SETTING_HOME = 20;
	static const int READY_TO_GO = 30;
	static const int PERFORMING_GO_TO = 100;
	static const int LEASHING = 140;
	static const int PAUSED = 150;
	static const int MANUAL_CNTRL = 1000;

	// STATE INITIALIZATION
	int currentState;
	int oldState;

	// int rate;
	bool new_state;
	// uint16_t counter_print;

	private:

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reference");
	ros::NodeHandle node;

	ReferenceNodeClass referenceNode(node);
    
	// referenceNode.Reference_Handle(); // run it just one time for initialization
	referenceNode.run();
	return 0;
}
