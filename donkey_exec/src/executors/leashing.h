// This may look like C code, but it is really -*- C++ -*-

#ifndef _EXECUTORLEASHING_H
#define _EXECUTORLEASHING_H

#include "executor.h"
#include "sherpa_msgs/Ack_mission.h"
#include "geographic_msgs/GeoPose.h"
#include "lrs_msgs_common/LeashingCommand.h"
#include "lrs_msgs_common/LeashingStatus.h"
#include "lrs_msgs_tst/ConfirmReq.h"
#include "sherpa_msgs/Cmd.h"
#include "sherpa_msgs/Ack_mission.h"
//#include "reference/LeashingCommand.h"   
//#include "reference/LeashingStatus.h"    
#include <wgs84_ned_lib/wgs84_ned_lib.h>  

#include "ros/ros.h"

#include <string>

namespace Exec {

  class Leashing : public virtual Executor {
  private:
	double desired_horizontal_distance;      //params
	double desired_vertical_distance;
	//double distance_xy;
	//int control_mode_xy;
	//double distance_altitude;
	//int control_altitude_mode;
	//double heading;
	//int heading_control_mode;
	bool enough_requested;      //enough flag to stop leashing
	bool pause_requested;
    bool continue_requested;
	bool flag_target_published;
	std::string target_position_topic;
	std::string leashing_command_topic;
	std::string leashing_status_topic;
	bool mission_succesfull;	//true if the command is accomplished  
	ros::Publisher leashing_target_pub;   //pubs are left here because
	ros::Publisher leashing_command_pub;  
	ros::Publisher leashing_status_pub;   
	int qqseq_;                   //internal seq to save the sequence of the saved command
	int max_time;                //max time for execution (ms)

  public:
    Leashing (std::string ns, int id);
    virtual ~Leashing () {};

	//virtual bool check ();
	bool prepare ();
	void start ();
	bool abort ();
	void callbackAckMission(const sherpa_msgs::Ack_mission::ConstPtr& msg);
	//void callbackLeashingTarget(const geographic_msgs::GeoPose::ConstPtr& msg);
	//void callbackLeashingCommand(const lrs_msgs_common::LeashingCommand::ConstPtr& msg);
	//void callbackLeashingStatus(const reference::LeashingStatus::ConstPtr& msg);
	virtual bool enough_execution();
	virtual bool request_pause ();
    virtual bool continue_execution ();
  };

};

#endif
