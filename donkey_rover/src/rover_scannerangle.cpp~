#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "libRover.h"
#include <cfloat>
#include <sstream>


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "rovstate");

  ros::NodeHandle n;


  ros::Publisher drivetl_pub = n.advertise<std_msgs::Float64>("rovstate", 1000);

  ros::Rate loop_rate(10);
  
  /* ROVER INITIALIZATION */

  Rover rover(false);
  int retCode = rover.init();
  if (retCode != 0)
		{
     	// Failed to init the rover
		return -1;
  }

    EScannerState state;
    while (state != ESSIdle)
    {
        cout << '.' << flush;
        Time::sleep(1, 0);
        retCode = rover.getScannerState(state);
    } 

  float angle=0;
    
 
 // Adjust the scanner home position.
            float adj, pos;

            adj = M_2PI*10.0/360.0; // Add 10°
            retCode = rover.setScannerAdjustment(adj);
            if (retCode != 0)
            {
                printError("setScannerAdjustment failed", retCode);
                return -1;
            }
    while (state != ESSIdle)
    {
        cout << '.' << flush;
        Time::sleep(1, 0);
        retCode = rover.getScannerState(state);
    } 

            // Change the roll angle.
            retCode = rover.setScannerAngle(M_PI/4.0);
            if (retCode != 0)
            {
                printError("setScannerAngle failed", retCode);
                return -1;
            }
        

        // Change the roll period.
        retCode = rover.setScannerPeriod(2.5);
        if (retCode != 0)
        {
            printError("setScannerPeriod failed", retCode);
            return -1;
        }

    while (state != ESSIdle)
    {
        cout << '.' << flush;
        Time::sleep(1, 0);
        retCode = rover.getScannerState(state);
    } 
      

        retCode = rover.sendScannerCommand(ESCStart);
        if (retCode != 0)
        {
            printError("Start failed", retCode);
            return -1;
        }
        cout << "* State =";
       
  while (ros::ok())
  {
    /*GETING THE ANGLE*/
    
    retCode = rover.getScannerAdjustment (angle);
    
    if (retCode != 0)
		{
     	// Failed to read the angle
		return -1;
    }
    std_msgs::Float64 msg;
    msg.data=angle;
    /*msg.data=obj.getScannerHomePosition( _pos);*/

    drivetl_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

 
            retCode = rover.sendScannerCommand(ESCStop);

 while (state != ESSIdle)
    {
        cout << '.' << flush;
        Time::sleep(1, 0);
        retCode = rover.getScannerState(state);
    } 
      
            retCode = rover.sendScannerCommand(ESCGoHome);
        
        if (retCode != 0)
        {
            printError("Stop/GoHome failed", retCode);
            return -1;
        }

         while (state != ESSIdle)
    {
        cout << '.' << flush;
        Time::sleep(1, 0);
        retCode = rover.getScannerState(state);
    } 
      


  return 0;
}
