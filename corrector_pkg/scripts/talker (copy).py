#!/usr/bin/env python
import rospy
import csv
import sys
import math
#from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

def talker():

    pub = rospy.Publisher('c1', Vector3, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	print ('Waiting for the scanner command (gohome, start (p), stop (s), insert (i): ');	
	name = raw_input()
	corrector = Vector3()
	corrector.y = 0.98
        if name == 'gohome':
	     corrector.z = 1.0
        if name == 'start' or name == 'p':
	     corrector.z = 2.0
        if name == 'stop' or name == 's':
	     corrector.z = 3.0	
	if name == 'insert' or name == 'i':
	     #angle = input("Please insert your angle in deg: ")
	     #corrector.x = math.radians(angle)
	     corrector.z = 4.0
	#corrector2 = 1.2        
	#hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        pub.publish(corrector)
        #pub2.publish(corrector2)	
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
