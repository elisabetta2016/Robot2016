#include "driveto.h"

#include <iostream>
#include <string>

#include "tstutil.h"
#include "executil.h"

extern std::map<std::string, boost::thread *> threadmap;

using namespace std;


Exec::DriveTo::DriveTo (std::string ns, int id) : Executor (ns, id) {
  set_delegation_expandable(false);
  lrs_msgs_tst::TSTExecInfo einfo;
  einfo.can_be_aborted = true;
  set_exec_info(ns, id, einfo);
}

void Exec::DriveTo::callbackAckMission(const std_msgs::Bool::ConstPtr& ack){
	mission_succesfull = true;
}

bool Exec::DriveTo::check () {
  ROS_INFO ("DriveTo CHECK");

  std::string ns = ros::names::clean (ros::this_node::getNamespace());

  fetch_node_info();

  if (!init_params ()) {
    ROS_ERROR("expand: init_params failed");
    return false;
  }

  return true;
}


bool Exec::DriveTo::prepare () {
  bool res = true;
  ROS_INFO ("Exec::DriveTo::prepare");
  if (res) {
    set_active_flag (node_ns, node_id, true);
  }
  return res;
}


void Exec::DriveTo::start () {
  ROS_INFO ("Exec::DriveTo::start: %s - %d", node_ns.c_str(), node_id);

  ros::NodeHandle n;
  ros::Subscriber mission_ack_sub = n.subscribe("/rover_ack_wp", 10, &Exec::DriveTo::callbackAckMission,this);  //subscriber to ack_mission topic
  ros::Publisher target_pub = n.advertise<geometry_msgs::Vector3>("/rover_wp", 10);

  try {

    if (!do_before_work()) {
      return;
    }

    double speed;

    if (!get_param("commanded-speed", speed)) {
      // Try to use the qualitative speed
      std::string qspeed;
      if (get_param("speed", qspeed)) {
	// Assign speed dependent on the value of qspeed
	if (qspeed == "slow") {
	  speed = 0.1;
	}
	if (qspeed == "standard") {
	  speed = 0.5;
	}
	if (qspeed == "fast") {
	  speed = 1.0;
	}
      } else {
	// Use default speed
	speed = 0.5;
      }
    } 

    geographic_msgs::GeoPoint gp;
    if (get_param("p", gp)) {
      ROS_INFO ("DRIVETO: %f %f - %f - %f", gp.latitude, gp.longitude, gp.altitude, speed);
    } else {
      fail ("driveTo: parameter p is missing");
      return;
    }

	target_vector3.x = gp.latitude;
	target_vector3.y = gp.longitude;
	target_vector3.z = 0;            //don't override
    //send target
	target_pub.publish(target_vector3);

    int max_time=60000;

	boost::this_thread::interruption_point();
	for (int i=0; i<max_time; i++) {
		usleep(1000);
		boost::this_thread::interruption_point();
		if (mission_succesfull) break;
	}


    wait_for_postwork_conditions ();
  }
  catch (boost::thread_interrupted) {
    abort_fail("driveto ABORT");
    return;
  }

}

bool Exec::DriveTo::abort () {
  bool res = false;
  ROS_INFO("Exec::DriveTo::abort");

  ostringstream os;
  os << node_ns << "-" << node_id;
  if (threadmap.find (os.str()) != threadmap.end()) {
    ROS_ERROR("EXECUTOR EXISTS: Sending interrupt to running thread");
    //    threadmap[os.str()]->interrupt();

    // Platform specific things to to

    return true;
  } else {
    ROS_ERROR ("Executor does not exist: %s", os.str().c_str());
    return false;
  }

  return res;
}
