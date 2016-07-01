// This may look like C code, but it is really -*- C++ -*-

#ifndef _DRIVETO_AT_H
#define _DRIVETO_AT_H

#include "executor.h"
#include "geoconvertros.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Bool.h"

#include <string>

namespace Exec {

  class DriveTo : public virtual Executor {
  private:
    //geometry_msgs::PoseStamped current_pose;
	geometry_msgs::Vector3 target_vector3;
	bool mission_succesfull;

  public:
    DriveTo (std::string ns, int id);
    virtual ~DriveTo () {};

    virtual void callbackAckMission(const std_msgs::Bool::ConstPtr& ack);

    virtual bool check ();

    virtual bool prepare ();
    virtual void start ();
    virtual bool abort ();
  };
};
#endif
