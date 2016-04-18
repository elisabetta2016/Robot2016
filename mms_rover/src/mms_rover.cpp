#include "ros/ros.h"

#include <sherpa_msgs/Cmd.h> // input
#include <sherpa_msgs/Ack_mission.h>// output
#include <sherpa_msgs/MMS_status.h>// output
#include <sherpa_msgs/Distance.h>// input
#include <sherpa_msgs/Safety.h>// input
#include <sherpa_msgs/LeashingStatus.h>// input
#include "geometry_msgs/Vector3.h"// input
#include <stdlib.h>

// STATES DEFINITION -> CREATE A DEDICATED LIBRARY = TODO
#define NO_HOME 10
#define SETTING_HOME 20
#define READY_TO_GO 30
#define PERFORMING_GO_TO 100
#define LEASHING 140
#define PAUSED 150
#define MANUAL_CNTRL 1000

const double PI = 3.1416; // pi
double eps_WP; // distance to the target WAYPOINT position in meters  //TODO no hardcoded
bool is_home_set = false;
int currentState = NO_HOME;
int previousState = NO_HOME;
float distance_body = 99999;
bool is_new_leashing = false;

class MmsNodeClass {
public:
	MmsNodeClass(ros::NodeHandle& node)
	{

		n_=node;

		//subscribers
		subFromCmd_=n_.subscribe("/sent_command", 10, &MmsNodeClass::readCmdMessage,this); 
		subFromDistance_=n_.subscribe("/distance", 10, &MmsNodeClass::readDistanceMessage,this);
		subFromBodyError_=n_.subscribe("/body_error", 10, &MmsNodeClass::readBodyErrorMessage,this);
		subLeashingStatus_ = n_.subscribe("/leashing_status", 10, &MmsNodeClass::readLeashingStatusMessage,this);
		subFromHome_= n_.subscribe("/home", 10, &MmsNodeClass::readHomeMessage,this);
		subLeashingTargetPosition_ = n_.subscribe("/leashing_target_position", 10, &MmsNodeClass::readLeashingTarget,this);
		
		// publishers
		pubToAckMission_=n_.advertise<sherpa_msgs::Ack_mission>("/ack_mission", 10);
		pubToMmsStatus_=n_.advertise<sherpa_msgs::MMS_status>("/mms_status", 10);
		pubCmd_ = n_.advertise<sherpa_msgs::Cmd>("/cmd_from_mms", 10);

		//Initializing outputAckMission_
		outputAckMission_.mission_item_reached = false;
		outputAckMission_.mav_mission_accepted = false;
		outputAckMission_.seq = 0;

		outputMmsStatus_.mms_state = NO_HOME;
		
		//Initializing states
		SET_HOME = false;
		//MISSION_START = false;
		WAYPOINT = false;
		SAFETY_ON = false;
		SAFETY_OFF = false;
		LEASHING_START = false;
		LEASHING_END = false;
		LEASHING_FAILURE = false;
		PAUSE = false;
		CONTINUE = false;

		//Init something
		currentState = NO_HOME;
		previousState = NO_HOME;
		// rate = 10;
		counter_print = 0;
		seq_number = 0;
	}
	
	void readLeashingTarget(const sherpa_msgs::GeoPoint::ConstPtr& msg)
	{
		leashing_target_ = *msg;
		//ROS_INFO("MMS: LEASHING TARGET RECEIVED");

		if ((leashing_target_.latitude != 0.0) && (leashing_target_.longitude != 0.0)) 
		{	
			if ((leashing_target_.latitude < 90.0) && (leashing_target_.latitude > -90.0) && (leashing_target_.longitude < 180.0) && (leashing_target_.longitude > -180.0))
			{
				// ROS_INFO("LEASHING TARGET VALID");
				is_new_leashing = true;
			}
		}
		else
		{
			//ROS_INFO("LEASHING TARGET NOT VALID");
			is_new_leashing = false;
		}
	}
	
	void readHomeMessage(const sherpa_msgs::GeoPoint::ConstPtr& msg)
	{
		inputHome_ = *msg;
		is_home_set = true;
		MMS_Handle();
	}
	
	void readDistanceMessage(const sherpa_msgs::Distance::ConstPtr& msg)
	{
		inputDist_.error_pos = msg->error_pos;
		//inputDist_.error_ang = msg->error_ang;
		//inputDist_.error_alt = msg->error_alt;
		inputDist_.command   = msg->command;
		inputDist_.seq       = msg->seq;
		MMS_Handle();
	}

	void readBodyErrorMessage(const geometry_msgs::Vector3::ConstPtr& msg)
	{
		inputBodyError_.x = msg->x;
		inputBodyError_.y = msg->y;
		inputBodyError_.z = msg->z;		
		distance_body = sqrt(pow(inputBodyError_.x,2)+pow(inputBodyError_.y,2)+pow(inputBodyError_.z,2));
		MMS_Handle();
	}
	
	void readSafetyMessage(const sherpa_msgs::Safety::ConstPtr& msg)
	{
		Safety_.safety = msg->safety;
		if (Safety_.safety && !SAFETY_ON)
		{
			SAFETY_ON = true;
			SAFETY_OFF = false;
			ROS_WARN("MMS: safety on");
		}
		else if (!Safety_.safety && !SAFETY_OFF)
		{
			SAFETY_OFF = true;
			SAFETY_ON = false;
			ROS_WARN("MMS: safety off");
		}
		MMS_Handle();
	}

	void readLeashingStatusMessage(const sherpa_msgs::LeashingStatus::ConstPtr& msg)
	{
		if (msg->failure > 0)
		{
			LEASHING_FAILURE = true;
		}
		MMS_Handle();
	}

	void readCmdMessage(const sherpa_msgs::Cmd::ConstPtr& msg)
	{
		inputCmd_.command = msg -> command;
		inputCmd_.param1  = msg -> param1;
		inputCmd_.param2  = msg -> param2;
		inputCmd_.param3  = msg -> param3;
		inputCmd_.param4  = msg -> param4;
		inputCmd_.param5  = msg -> param5;
		inputCmd_.param6  = msg -> param6;
		inputCmd_.param7  = msg -> param7;
		inputCmd_.frame  = msg -> frame;
		inputCmd_.seq  = msg -> seq;

        ROS_INFO("MMS: CMD_RECEIVED %d. Sequence: %d", inputCmd_.command, inputCmd_.seq);

		switch(inputCmd_.command)
		{
			case 16:  // MAV_CMD_NAV_WAYPOINT
			{
				ROS_INFO("MMS: CMD_WAYPOINT. Params: %f - %f - %f - %f",inputCmd_.param5,inputCmd_.param6,inputCmd_.param7,inputCmd_.param4);
				seq_number = inputCmd_.seq;
				WAYPOINT = true;
				// pubCmd_.publish(inputCmd_);  //cmd passed to reference
			}break;

			case 179: // MAV_CMD_DO_SET_HOME
			{
				ROS_INFO("MMS: CMD_SET_HOME");
				SET_HOME = true;
				// pubCmd_.publish(inputCmd_);  //cmd passed to reference
				seq_number = inputCmd_.seq;
			}break;

			case 25:  // MAV_CMD_NAV_FOLLOW (LEASHING)
			{
				//seq_number = inputCmd_.seq;
				if (inputCmd_.param1 == 1)
				{
					ROS_INFO("MMS: CMD_LEASHING_START");
					if (is_new_leashing)
					{
						ROS_INFO("MMS: LEASHING TARGET VALID");
						LEASHING_START = true;					
					}
					else
					{
						ROS_INFO("MMS: LEASHING TARGET NOT VALID");
					}
					// pubCmd_.publish(inputCmd_);  //cmd passed to reference
				} else if (inputCmd_.param1 == 0)
				{
					LEASHING_END = true;
					ROS_INFO("MMS: CMD_LEASHING_STOP");
					// pubCmd_.publish(inputCmd_);  //cmd passed to reference
				} else
				{
					outputAckMission_.mission_item_reached = false;
					outputAckMission_.seq = seq_number;
					outputAckMission_.mav_mission_accepted = false;
					pubToAckMission_.publish(outputAckMission_);
					ROS_WARN("MMS->GCS: MISSION_ITEM_NOT_ACCEPTED (LEASHING). WRONG PARAM1");
				}
				
			}break;

			case 252:  // MAV_CMD_OVERRIDE_GOTO (PAUSE/CONTINUE)
			{
				//seq_number = inputCmd_.seq;
				if (inputCmd_.param1 == 0){          //PAUSE
					PAUSE = true;
					ROS_INFO("MMS: CMD_PAUSE");
					pubCmd_.publish(inputCmd_);  //cmd passed to reference
				} else if (inputCmd_.param1 == 1){		//CONTINUE
					CONTINUE = true;
					ROS_INFO("MMS: CMD_CONTINUE");
					pubCmd_.publish(inputCmd_);  //cmd passed to reference
				} else {
					outputAckMission_.mission_item_reached = false;
					outputAckMission_.seq = seq_number;
					outputAckMission_.mav_mission_accepted = false;
					pubToAckMission_.publish(outputAckMission_);
					ROS_WARN("MMS->GCS: MISSION_ITEM_NOT_ACCEPTED (PAUSE/CONTINUE). WRONG PARAM1");
				}
			}break;
		}
		MMS_Handle();
	}

	void set_events_false()
	{
		WAYPOINT = false;
		SAFETY_ON = false;
		SAFETY_OFF = false;
		LEASHING_START = false;
		LEASHING_END = false;
		LEASHING_FAILURE = false;
		PAUSE = false;
		CONTINUE = false;
		is_home_set = false;
		SET_HOME = false;
	}

	bool check_events_true()
	{
		bool is_events;
		is_events = (WAYPOINT || SAFETY_ON || LEASHING_START || LEASHING_END || LEASHING_FAILURE || PAUSE || CONTINUE);
		return is_events;
	}
	
	void MMS_Handle()
	{
		switch(currentState)

		{

			case NO_HOME:
				/*outputMmsStatus_.mms_state = currentState;
				pubToMmsStatus_.publish(outputMmsStatus_);
				ROS_INFO("MMS->REF: CURRENT_STATE = NO_HOME");*/
				SET_HOME = true;
				
				if (SAFETY_ON)
				{                    
					//set_events_false();
					previousState = currentState; 
					currentState = MANUAL_CNTRL;
					outputMmsStatus_.mms_state = currentState;
					pubToMmsStatus_.publish(outputMmsStatus_);
					ROS_INFO("MMS->REF: NEXT_STATE = MANUAL_CNTRL");
				}				
				
				if (SET_HOME && !SAFETY_ON)
				{
					/*// ACK THE RECEIVED COMMAND
					outputAckMission_.mission_item_reached = false;
					outputAckMission_.seq = inputCmd_.seq;
					outputAckMission_.mav_mission_accepted = true;
					pubToAckMission_.publish(outputAckMission_);
					ROS_INFO("MMS->GCS: MISSION_ACCEPTED");
					// PUBLISH THE RECEIVED CMD
					pubCmd_.publish(inputCmd_);  //cmd passed to reference*/

					// DO STATE TRANSITION
					currentState = SETTING_HOME;
					outputMmsStatus_.mms_state = currentState;
					// PUBLISH THE MACHINE STATE
					pubToMmsStatus_.publish(outputMmsStatus_);
					ROS_INFO("MMS->REF: NEXT_STATE = SETTING_HOME");
				}
				
				if (check_events_true() && !SET_HOME && !SAFETY_ON)
				{
					// RESET THE EVENTS
					set_events_false();
					// ACK THE RECEIVED COMMAND
					outputAckMission_.mission_item_reached = false;
					outputAckMission_.seq = inputCmd_.seq;
					outputAckMission_.mav_mission_accepted = false;
					pubToAckMission_.publish(outputAckMission_);
					ROS_WARN("MMS->GCS: MISSION_NOT_ACCEPTED");
				}
				// RESET THE EVENTS
				set_events_false();
			break;

			case SETTING_HOME:
				//ROS_INFO("MMS->REF: CURRENT_STATE = SETTING_HOME");
				//ROS_INFO("is_home_set %d", is_home_set);

				if (SAFETY_ON)
				{                    
					//set_events_false();
					previousState = currentState; 
					currentState = MANUAL_CNTRL;
					outputMmsStatus_.mms_state = currentState;
					pubToMmsStatus_.publish(outputMmsStatus_);
					ROS_INFO("MMS->REF: NEXT_STATE = MANUAL_CNTRL");
				}		
				
				if (is_home_set && !SAFETY_ON)
				{

					// ACK THE RECEIVED COMMAND
					outputAckMission_.mission_item_reached = true;
					outputAckMission_.seq = seq_number;
					outputAckMission_.mav_mission_accepted = false;
					pubToAckMission_.publish(outputAckMission_);
					ROS_INFO("MMS->GCS: MISSION_ITEM_REACHED");

					currentState = READY_TO_GO;
					outputMmsStatus_.mms_state = currentState;
					pubToMmsStatus_.publish(outputMmsStatus_);
					ROS_INFO("MMS->REF: NEXT_STATE = READY_TO_GO");					
				}
				// RESET THE EVENTS
				set_events_false();
			break;

			case READY_TO_GO:
				//ROS_INFO("MMS->REF: CURRENT_STATE = READY_TO_GO");
				
				if (SAFETY_ON)
				{                    
					//set_events_false();
					previousState = currentState; 
					currentState = MANUAL_CNTRL;
					outputMmsStatus_.mms_state = currentState;
					pubToMmsStatus_.publish(outputMmsStatus_);
					ROS_INFO("MMS->REF: NEXT_STATE = MANUAL_CNTRL");
				}
				if (WAYPOINT && !SAFETY_ON)
				{
					//set_events_false();
					// ACK THE RECEIVED COMMAND
					outputAckMission_.mission_item_reached = false;
					outputAckMission_.seq = inputCmd_.seq;
					outputAckMission_.mav_mission_accepted = true;
					pubToAckMission_.publish(outputAckMission_);
					ROS_INFO("MMS->GCS: MISSION_ACCEPTED");
					// PUBLISH THE RECEIVED CMD
					pubCmd_.publish(inputCmd_);  //cmd passed to reference
					
					currentState = PERFORMING_GO_TO; 
					outputMmsStatus_.mms_state = currentState;
					pubToMmsStatus_.publish(outputMmsStatus_);
					ROS_INFO("MMS->REF: NEXT_STATE = PERFORMING_GO_TO");
				}
				if (LEASHING_START && !WAYPOINT && !SAFETY_ON) 
				{
					//set_events_false();
					// ACK THE RECEIVED COMMAND
					outputAckMission_.mission_item_reached = false;
					outputAckMission_.seq = inputCmd_.seq;
					outputAckMission_.mav_mission_accepted = true;
					pubToAckMission_.publish(outputAckMission_);
					ROS_INFO("MMS->GCS: MISSION_ACCEPTED");
					// PUBLISH THE RECEIVED CMD
					pubCmd_.publish(inputCmd_);  //cmd passed to reference
					
					currentState = LEASHING;
					outputMmsStatus_.mms_state = currentState;
					pubToMmsStatus_.publish(outputMmsStatus_);
					ROS_INFO("MMS->REF: NEXT_STATE = LEASHING");
				}
				if (SET_HOME && !LEASHING_START && !WAYPOINT && !SAFETY_ON)
				{
					// ACK THE RECEIVED COMMAND
					outputAckMission_.mission_item_reached = false;
					outputAckMission_.seq = inputCmd_.seq;
					outputAckMission_.mav_mission_accepted = true;
					pubToAckMission_.publish(outputAckMission_);
					ROS_INFO("MMS->GCS: MISSION_ACCEPTED");
					// PUBLISH THE RECEIVED CMD
					pubCmd_.publish(inputCmd_);  //cmd passed to reference

					//set_events_false();
					currentState = SETTING_HOME;
					outputMmsStatus_.mms_state = currentState;
					pubToMmsStatus_.publish(outputMmsStatus_);
					ROS_INFO("MMS->REF: NEXT_STATE = SETTING_HOME");
				}
				if (check_events_true() && !SET_HOME && !LEASHING_START && !WAYPOINT && !SAFETY_ON)
				{
					//set_events_false();
					// ACK THE RECEIVED COMMAND
					outputAckMission_.mission_item_reached = false;
					outputAckMission_.seq = inputCmd_.seq;
					outputAckMission_.mav_mission_accepted = true;
					pubToAckMission_.publish(outputAckMission_);
					ROS_WARN("MMS->GCS: MISSION_NOT_ACCEPTED");
				}
				// RESET THE EVENTS
				set_events_false();
				break;

			case PERFORMING_GO_TO:
				// ROS_INFO("MMS->REF: CURRENT_STATE = PERFORMING_GO_TO");
				if (SAFETY_ON)
				{    
					//set_events_false();
					previousState = currentState;
					currentState = MANUAL_CNTRL;
					outputMmsStatus_.mms_state = currentState;
					pubToMmsStatus_.publish(outputMmsStatus_);
					ROS_INFO("MMS->REF: NEXT_STATE = MANUAL_CNTRL");
				}
				if (PAUSE && !SAFETY_ON)
				{
					//set_events_false();
					previousState = currentState;   //save last state in previousState
					currentState = PAUSED;
					outputMmsStatus_.mms_state = currentState;
					pubToMmsStatus_.publish(outputMmsStatus_);
					ROS_WARN("MMS->REF: NEXT_STATE = PAUSED");
				}
				if (WAYPOINT && !PAUSE && !SAFETY_ON)
				{
					//set_events_false();
					// ACK THE RECEIVED COMMAND
					outputAckMission_.mission_item_reached = false;
					outputAckMission_.seq = inputCmd_.seq;
					outputAckMission_.mav_mission_accepted = true;
					pubToAckMission_.publish(outputAckMission_);
					ROS_INFO("MMS->GCS: MISSION_ACCEPTED");
					// PUBLISH THE RECEIVED CMD
					pubCmd_.publish(inputCmd_);  //cmd passed to reference
					
					currentState = PERFORMING_GO_TO;
					outputMmsStatus_.mms_state = currentState;
					pubToMmsStatus_.publish(outputMmsStatus_);
					ROS_INFO("MMS->REF: NEXT_STATE = PERFORMING_GO_TO");
				}
				if (inputDist_.command == 16  && seq_number == inputDist_.seq && !PAUSE && !WAYPOINT && !SAFETY_ON)
				{
					ROS_INFO_ONCE("MMS: REACHING THE WAYPOINT TARGET");

					if (distance_body < eps_WP)
					{
						//set_events_false();
						// ACK THE RECEIVED COMMAND
						outputAckMission_.mission_item_reached = true;
						outputAckMission_.seq = seq_number;
						outputAckMission_.mav_mission_accepted = false;
						pubToAckMission_.publish(outputAckMission_);
						ROS_INFO("MMS->GCS: MISSION_ITEM_REACHED");

						currentState = READY_TO_GO;
						outputMmsStatus_.mms_state = currentState;
						pubToMmsStatus_.publish(outputMmsStatus_);
						ROS_INFO("MMS->REF: NEXT_STATE = READY_TO_GO");
					}
				}				
				if (check_events_true() && !PAUSE && !WAYPOINT && !SAFETY_ON)
				{
					//set_events_false();
					// ACK THE RECEIVED COMMAND
					outputAckMission_.mission_item_reached = false;
					outputAckMission_.seq = inputCmd_.seq;
					outputAckMission_.mav_mission_accepted = false;
					pubToAckMission_.publish(outputAckMission_);
					ROS_WARN("MMS->GCS: MISSION_NOT_ACCEPTED");
				}
				// RESET THE EVENTS
				set_events_false();
			break;
		
			case MANUAL_CNTRL:
				// ROS_INFO("MMS->REF: CURRENT_STATE = MANUAL_CNTRL");
				if (SAFETY_OFF)
				{
					//set_events_false();
					currentState = previousState; 
					outputMmsStatus_.mms_state = currentState;
					pubToMmsStatus_.publish(outputMmsStatus_);
				}
				// RESET THE EVENTS
				set_events_false();
			break;
				
			case PAUSED:
				// ROS_INFO("MMS->REF: CURRENT_STATE = PAUSED");
				if (SAFETY_ON)
				{                    
					//set_events_false();
					previousState = currentState; 
					currentState = MANUAL_CNTRL;
					outputMmsStatus_.mms_state = currentState;
					pubToMmsStatus_.publish(outputMmsStatus_);
					ROS_INFO("MMS->REF: NEXT_STATE = MANUAL_CNTRL");
				}

				if (CONTINUE && !SAFETY_ON)
				{
					//set_events_false();
					currentState = previousState;   
					outputMmsStatus_.mms_state = currentState;
					pubToMmsStatus_.publish(outputMmsStatus_);
				}
				// RESET THE EVENTS
				set_events_false();
				break;
			
			case LEASHING:
				//ROS_INFO("MMS->REF: CURRENT_STATE = LEASHING");
				if (SAFETY_ON)
				{    
					//set_events_false();
					previousState = currentState;
					currentState = MANUAL_CNTRL;
					outputMmsStatus_.mms_state = currentState;
					pubToMmsStatus_.publish(outputMmsStatus_);
					ROS_INFO("MMS->REF: NEXT_STATE = MANUAL_CNTRL");
				}
				if (PAUSE && !SAFETY_ON)
				{
					//set_events_false();
					previousState = currentState;   //save last state in previousState
					currentState = PAUSED;
					outputMmsStatus_.mms_state = currentState;
					pubToMmsStatus_.publish(outputMmsStatus_);
					ROS_INFO("MMS->REF: NEXT_STATE = PAUSED");
				}
				if (LEASHING_END && !PAUSE && !SAFETY_ON)
				{
					//set_events_false();
					// ACK THE RECEIVED COMMAND
					outputAckMission_.mission_item_reached = true;
					outputAckMission_.seq = inputCmd_.seq;
					outputAckMission_.mav_mission_accepted = false;
					pubToAckMission_.publish(outputAckMission_);
					ROS_INFO("MMS->GCS: MISSION_ACCEPTED (END LEASHING)");
					// PUBLISH THE RECEIVED CMD
					pubCmd_.publish(inputCmd_);  //cmd passed to reference
					
					currentState = READY_TO_GO;
					outputMmsStatus_.mms_state = currentState;
					pubToMmsStatus_.publish(outputMmsStatus_);
					ROS_INFO("MMS->REF: NEXT_STATE = READY_TO_GO");
				}
				if (LEASHING_FAILURE && !PAUSE && !SAFETY_ON)
				{
					//set_events_false();
					// ACK THE RECEIVED COMMAND
					outputAckMission_.mission_item_reached = false;
					outputAckMission_.seq = inputCmd_.seq;
					outputAckMission_.mav_mission_accepted = false;
					pubToAckMission_.publish(outputAckMission_);
					ROS_ERROR("MMS->GCS: LEASHING_FAILED");

					currentState = READY_TO_GO;
					outputMmsStatus_.mms_state = currentState;
					pubToMmsStatus_.publish(outputMmsStatus_);
					ROS_INFO("MMS->REF: NEXT_STATE = READY_TO_GO");
				}
				// RESET THE EVENTS
				set_events_false();
				break;
		}
}

void run()
{
	ROS_INFO("MMS: RUNNING");
	outputMmsStatus_.mms_state = currentState;
	pubToMmsStatus_.publish(outputMmsStatus_);
	ROS_INFO("MMS->REF: CURRENT_STATE = NO_HOME");
	ros::NodeHandle n("~");
	n.param("WP_tracking_precision", eps_WP, 1.0);
	ROS_INFO_ONCE("MMS: esp_WP = %f",eps_WP);
	ros::spin();
	/*ros::Rate loop_rate(rate);

	while (ros::ok())
	{
		ROS_INFO("MMS: RUNNING");

		MMS_Handle();
		ros::spinOnce();

		loop_rate.sleep();
	}*/
}

protected:
/*state here*/
ros::NodeHandle n_;



// Subscribers

ros::Subscriber subFromCmd_;
sherpa_msgs::Cmd inputCmd_;

ros::Subscriber subSafety_;
sherpa_msgs::Safety Safety_;

ros::Subscriber subLeashingStatus_;

ros::Subscriber subFromDistance_;
sherpa_msgs::Distance inputDist_;

ros::Subscriber subFromHome_;
sherpa_msgs::GeoPoint inputHome_;

ros::Subscriber subFromBodyError_;
geometry_msgs::Vector3 inputBodyError_;

// Publishers
ros::Publisher pubToAckMission_;
sherpa_msgs::Ack_mission outputAckMission_;

ros::Publisher pubCmd_;

ros::Publisher pubToMmsStatus_;
sherpa_msgs::MMS_status outputMmsStatus_;

ros::Subscriber subLeashingTargetPosition_;
sherpa_msgs::GeoPoint leashing_target_;	 //leashing

bool SET_HOME;
// bool MISSION_START;
bool WAYPOINT;
bool SAFETY_ON;
bool SAFETY_OFF;
bool LEASHING_START;
bool LEASHING_END;
bool LEASHING_FAILURE;
bool PAUSE;
bool CONTINUE;

// STATE INITIALIZATION
// int currentState;
// int previousState;

// int rate;



private:

uint16_t counter_print;
uint16_t seq_number;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mms");
	
	ros::NodeHandle node;

	MmsNodeClass mmsNode(node);
	mmsNode.run();
	
	return 0;
}
