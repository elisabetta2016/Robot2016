#!/usr/bin/env python

import os.path
import rospy
import sys

from geometry_msgs.msg import Vector3

def sendTestWps(argv):
    print "[%s:] Test W/Ps Transmission Initialising ..." % os.path.basename(argv[0])
    rospy.init_node('sendTestWps', anonymous=True)
    wpPub = rospy.Publisher('rover_wp', Vector3, queue_size=10)
    wpRate = rospy.Rate(1)
    print "[%s:] Test W/Ps Transmission Ready!" % os.path.basename(argv[0])
    while not rospy.is_shutdown():
 	print ('Insert here the new W/P latitude (DEG), longitude (DEG), overwrite flag (0 off, 1 on)): ');	
	wpInput = raw_input()
        wpLat, wpLon, wpOverWrite = wpInput.strip().split(',')
        waypoint = Vector3()
        waypoint.x = float(wpLat)#44.153278#
        waypoint.y = float(wpLon)#12.241426#
        waypoint.z = float(wpOverWrite)# 0.0#
        wpPub.publish(waypoint)
        if waypoint.z > 0:
            print "[%s:] Sent W/P latitude %.6f DEG, longitude %.6f DEG, overwrite on" % (os.path.basename(argv[0]),waypoint.x,waypoint.y)
        else:
            print "[%s:] Sent W/P latitude %.6f DEG, longitude %.6f DEG, overwrite off" % (os.path.basename(argv[0]),waypoint.x,waypoint.y)
        wpRate.sleep()
        

if __name__ == '__main__':
    try:
        sendTestWps(sys.argv)
    except rospy.ROSInterruptException:
        pass
