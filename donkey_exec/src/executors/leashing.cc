#include "leashing.h"

#include "ros/ros.h"

#include <boost/thread.hpp>
#include <iostream>
 

#include "executil.h"

extern ros::NodeHandle * global_nh;
extern ros::Publisher * global_confirm_pub;
extern ros::Publisher * global_cmd_pub;

extern std::map<std::string, boost::thread *> threadmap;

extern int16_t global_seq;    //TODO check if needed
extern boost::mutex seq_mutex;  //TODO check if needed
extern boost::mutex ref_mutex;  //TODO check if needed

using namespace std;

Exec::Leashing::Leashing (std::string ns, int id) : Executor (ns, id) {
	lrs_msgs_tst::TSTExecInfo einfo;
	einfo.can_be_aborted = true;
	einfo.can_be_enoughed = true;
	einfo.can_be_paused = true;
	set_exec_info(ns, id, einfo);	
}


/*void Exec::Leashing::callbackAckMission(const sherpa_msgs::Ack_mission::ConstPtr& msg){ //don't think leashing will have ack, but just TODO check for seq with low level
	ROS_INFO ("Exec::Leashing::Received ack. Sequence: %d - Mission item: %d",msg->seq, msg->mission_item_reached);
		if (msg->mission_item_reached && (msg->seq == qqseq_) && (qqseq_ != 0)){
		mission_succesfull = true;
	}
	if (!msg->mav_mission_accepted && (msg->seq == qqseq_) && (qqseq_ != 0)){
		fail ("Leashing: action not possible in actual state");
		return;
	}
}*/

/*void Exec::Leashing::callbackLeashingTarget(const geographic_msgs::GeoPose::ConstPtr& msg){
	//ROS_ERROR ("Exec::Leashing:: pub target topic");
	leashing_target_pub.publish(*msg);    //takes from lrs_msgs and publish to reference. To have separation between DF and high level SHERPA
	flag_target_published = true;
}*/

/*void Exec::Leashing::callbackLeashingCommand(const lrs_msgs_common::LeashingCommand::ConstPtr& msg){
	reference::LeashingCommand temp_leashing_command;
	temp_leashing_command.horizontal_distance = msg->horizontal_distance;
	temp_leashing_command.horizontal_distance_vel = msg->horizontal_distance_vel;
	temp_leashing_command.horizontal_heading = msg->horizontal_heading;
	temp_leashing_command.horizontal_heading_vel = msg->horizontal_heading_vel;
	temp_leashing_command.distance_north = msg->distance_north;
	temp_leashing_command.distance_north_vel = msg->distance_north_vel;
	temp_leashing_command.distance_east = msg->distance_east;
	temp_leashing_command.distance_east_vel = msg->distance_east_vel;
	temp_leashing_command.horizontal_control_mode = msg->horizontal_control_mode;
	temp_leashing_command.vertical_distance = msg->vertical_distance;
	temp_leashing_command.vertical_distance_vel = msg->vertical_distance_vel;
	temp_leashing_command.vertical_control_mode = msg->vertical_control_mode;
	temp_leashing_command.yaw = msg->yaw;
	temp_leashing_command.yaw_vel = msg->yaw_vel;
	temp_leashing_command.yawpoint = msg->yawpoint;
	temp_leashing_command.yaw_control_mode = msg->yaw_control_mode;
	//ROS_INFO ("Exec::Leashing:: pub leashing command topic");
	leashing_command_pub.publish(temp_leashing_command);		//takes from lrs_msgs and publish to reference. To have separation between DF and high level SHERPA
}*/

/*void Exec::Leashing::callbackLeashingStatus(const reference::LeashingStatus::ConstPtr& msg){
	lrs_msgs_common::LeashingStatus temp_leashing_status;
	temp_leashing_status.horizontal_control_mode = msg->horizontal_control_mode;
	temp_leashing_status.horizontal_distance = msg->horizontal_distance;
	temp_leashing_status.horizontal_heading = msg->horizontal_heading;
	temp_leashing_status.distance_north = msg->distance_north;
	temp_leashing_status.distance_east = msg->distance_east;
	temp_leashing_status.vertical_control_mode = msg->vertical_control_mode;
	temp_leashing_status.vertical_distance = msg->vertical_distance;
	temp_leashing_status.yaw_control_mode = msg->yaw_control_mode;
	temp_leashing_status.yaw = msg->yaw;
	temp_leashing_status.yawpoint = msg->yawpoint;
	//ROS_INFO ("Exec::Leashing:: pub leashing status topic");
	leashing_status_pub.publish(temp_leashing_status);		//takes from reference and publish to lrs_msgs. To have separation between DF and high level SHERPA
}*/

bool Exec::Leashing::prepare () {
  bool res = true;
  qqseq_ = 0;
  ROS_INFO ("Exec::Leashing::prepare");
  mission_succesfull = false;  
  flag_target_published = false;       //TODO do something to check if there is the target
  pause_requested = false;
  continue_requested = false;

  if (res) {
    set_active_flag (node_ns, node_id, true);
  }

  return res;
}


void Exec::Leashing::start () {
	//ros::Subscriber mission_ack_sub; 
	//ros::Subscriber leashing_target_sub;  
	//ros::Subscriber leashing_command_sub;  
	//ros::Subscriber leashing_status_sub; 
	
    ROS_INFO ("Exec::Leashing::start: %s - %d", node_ns.c_str(), node_id);
	//mission_ack_sub = global_nh->subscribe("/ack_mission", 10, &Exec::Leashing::callbackAckMission,this);
    try {

        if (!do_before_work ()) {
          return;
        }

		if (!get_param("position-topic", target_position_topic)) {
		    fail ("Leashing: parameter target_position_topic is missing");
		    return;
	    } else {
			ROS_INFO ("Exec::Leashing: position topic: %s", target_position_topic.c_str());
		}
		if (!get_param("command-topic", leashing_command_topic)) {
		    fail ("Leashing: parameter command-topic is missing");
		    return;
	    } else {
			ROS_INFO ("Exec::Leashing: command topic: %s", leashing_command_topic.c_str());
		}
		if (!get_param("status-topic", leashing_status_topic)) {
		    fail ("Leashing: parameter status-topic is missing");
		    return;
	    } else {
			ROS_INFO ("Exec::Leashing: status topic: %s", leashing_status_topic.c_str());
		}
		
		//leashing_target_sub = global_nh->subscribe(target_position_topic, 10, &Exec::Leashing::callbackLeashingTarget,this);
		//leashing_command_sub = global_nh->subscribe(leashing_command_topic, 10, &Exec::Leashing::callbackLeashingCommand,this);
		//leashing_status_sub = global_nh->subscribe("/leashing_status", 10, &Exec::Leashing::callbackLeashingStatus,this);  //received from reference
		//leashing_target_pub = global_nh->advertise<geographic_msgs::GeoPose>("/leashing_target_position",10);
		//leashing_command_pub = global_nh->advertise<reference::LeashingCommand>("/leashing_command",10);
		//leashing_status_pub = global_nh->advertise<lrs_msgs_common::LeashingStatus>(leashing_status_topic,10);  //send back to tst
		
	
        ROS_INFO ("Exec::Leashing: Execution unit: %s", tni.execution_ns.c_str());
		int counter_target = 0;
		while (!flag_target_published){   //need to publish target before sending leashing command to low level
			usleep (100000);   //0.1s
			counter_target++;
			if (counter_target > 50){   //5 seconds
				fail ("Leashing: no target position published");
				return;
			}
		}
		usleep(500000);  //wait half a second before issuing the command

        sherpa_msgs::Cmd cmd;

	    cmd.target_system = 0;          
	    cmd.target_component = 0;
	    {                                              //brackets are to ensure (I hope) that after the shared resource (seq) is accessed, the mutex destructor is called and the mutex unlocked
		    boost::mutex::scoped_lock lock(seq_mutex);
		    qqseq_ = global_seq;
		    cmd.seq = global_seq++;
	    }
	    ROS_INFO ("Sequence for Leashing: %d",qqseq_);
	    cmd.frame = 6;
	    cmd.command = 25;       			//MAV_CMD_NAV_FOLLOW         
	    cmd.current = 0;					//not used
	    cmd.autocontinue = 0;				//not used
	    cmd.param1 = 1;         			//1-->Start leashing; 0-->Stop leashing
	    cmd.param2 = 0;        				//NOT USED
	    cmd.param3 = 0;        				//NOT USED
		cmd.param4 = 0;        				//RESERVED
	    cmd.param5 = 0;    					//RESERVED
	    cmd.param6 = 0;	   					//RESERVED
	    cmd.param7 = 0;						//RESERVED

	    global_cmd_pub->publish (cmd);
	    ROS_INFO ("Exec::Leashing: Sent start command");
		
		max_time = 1000000;    //no max time....only enough

	    while (!enough_requested) {   //waiting for enough
		  usleep (100000);
		  boost::this_thread::interruption_point();
		}
		mission_succesfull = true;
		
		//Send command to stop leashing
	    cmd.param1 = 0;         			//1-->Start leashing; 0-->Stop leashing
		cmd.seq = 0;
	    global_cmd_pub->publish (cmd);
	    ROS_INFO ("Exec::Leashing: Sent stop command");
		
	    //
	    // When we reach this point the node execution would be finished.
	    //
        
	    ROS_INFO ("Exec::Leashing: FINISHED");

	    wait_for_postwork_conditions ();
    }
    catch (boost::thread_interrupted) {
        ROS_ERROR("BOOST INTERUPTED IN Leashing");
        set_succeeded_flag (node_ns, node_id, false);
        set_aborted_flag (node_ns, node_id, true);
        set_finished_flag (node_ns, node_id, true);
    }
}

bool Exec::Leashing::abort () {
  bool res = false;
  ROS_INFO("Exec::Leashing::abort");
  //Send command to stop leashing
	sherpa_msgs::Cmd cmd;
	cmd.target_system = 0;          
	cmd.target_component = 0;
	cmd.seq = 0;
	ROS_INFO ("Sequence for Leashing: %d",qqseq_);
	cmd.frame = 6;
	cmd.command = 25;       			//MAV_CMD_NAV_FOLLOW         
	cmd.current = 0;					//not used
	cmd.autocontinue = 0;				//not used
	cmd.param1 = 0;         			//1-->Start leashing; 0-->Stop leashing
	cmd.param2 = 0;        				//NOT USED
	cmd.param3 = 0;        				//NOT USED
	cmd.param4 = 0;        				//RESERVED
	cmd.param5 = 0;    					//RESERVED
	cmd.param6 = 0;	   					//RESERVED
	cmd.param7 = 0;						//RESERVED
	global_cmd_pub->publish (cmd);
	ROS_INFO ("Exec::Leashing: Sent stop command because abort");
	
  ostringstream os;
  os << node_ns << "-" << node_id;
  if (threadmap.find (os.str()) != threadmap.end()) {                               //PUT AGAIN
    ROS_ERROR("EXECUTOR EXISTS: Sending interrupt to running thread");
    threadmap[os.str()]->interrupt();
    // Platform specific things to to

    return true;
  } else {
    ROS_ERROR ("Executor does not exist: %s", os.str().c_str());
    return false;
  }
  return res;
}

bool Exec::Leashing::enough_execution () {
  bool res = true;
  ROS_ERROR ("Exec::ScanGroundSingle::enough_execution");
  enough_requested = true;
  return res;
}

bool Exec::Leashing::request_pause () {
	bool res = true;
	if (!pause_requested){
		continue_requested = false;
		set_paused_flag (node_ns, node_id, true);
		ROS_INFO ("Exec::Leashing::request_pause");
		//Send command to stop leashing
		sherpa_msgs::Cmd cmd;
		cmd.target_system = 0;          
		cmd.target_component = 0;
		cmd.seq = 0;
		ROS_INFO ("Sequence for Leashing: %d",qqseq_);
		cmd.frame = 6;
		cmd.command = 25;       			//MAV_CMD_NAV_FOLLOW         
		cmd.current = 0;					//not used
		cmd.autocontinue = 0;				//not used
		cmd.param1 = 0;         			//1-->Start leashing; 0-->Stop leashing
		cmd.param2 = 0;        				//NOT USED
		cmd.param3 = 0;        				//NOT USED
		cmd.param4 = 0;        				//RESERVED
		cmd.param5 = 0;    					//RESERVED
		cmd.param6 = 0;	   					//RESERVED
		cmd.param7 = 0;						//RESERVED
		global_cmd_pub->publish (cmd);
		ROS_INFO ("Exec::Leashing: Sent stop command because pause");
		pause_requested = true;
	}
	return res;
}

bool Exec::Leashing::continue_execution () {
	bool res = true;
	if (pause_requested && !continue_requested){
		pause_requested = false;
		set_paused_flag (node_ns, node_id, false);
		ROS_INFO ("Exec::Leashing::request_continue");
		//Send command to start leashing
		sherpa_msgs::Cmd cmd;
		cmd.target_system = 0;          
		cmd.target_component = 0;
		{                                              //brackets are to ensure (I hope) that after the shared resource (seq) is accessed, the mutex destructor is called and the mutex unlocked
		    boost::mutex::scoped_lock lock(seq_mutex);
		    qqseq_ = global_seq;
		    cmd.seq = global_seq++;
	    }
		ROS_INFO ("Sequence for Leashing: %d",qqseq_);
		cmd.frame = 6;
		cmd.command = 25;       			//MAV_CMD_NAV_FOLLOW         
		cmd.current = 0;					//not used
		cmd.autocontinue = 0;				//not used
		cmd.param1 = 1;         			//1-->Start leashing; 0-->Stop leashing
		cmd.param2 = 0;        				//NOT USED
		cmd.param3 = 0;        				//NOT USED
		cmd.param4 = 0;        				//RESERVED
		cmd.param5 = 0;    					//RESERVED
		cmd.param6 = 0;	   					//RESERVED
		cmd.param7 = 0;						//RESERVED
		global_cmd_pub->publish (cmd);
		ROS_INFO ("Exec::Leashing: Sent stop command because pause");
		continue_requested = true;
	}
	ROS_ERROR ("Exec::Leashing::request_continue");
	return res;
}
