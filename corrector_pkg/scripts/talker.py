#!/usr/bin/env python
import rospy
import csv
import sys
import math
import tf
#from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from donkey_rover.msg import Scanner_Command
from sensor_msgs.msg import Imu



#def imu_print(data):
#    q = tf.Quaternion(data.quaternion.x,data.quaternion.y,data.quaternion.z,data.quaternion.w)
#    m = tf.Matrix3x3(q)   
#    m.getRPY(roll,pitch,yaw)
#    print (yaw)


def talker():

    #rospy.Subscriber("imu/data_raw", Imu, imu_print) 
    pub = rospy.Publisher('scanner_commands', Scanner_Command, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	msg = Scanner_Command()
	msg.Scanner_Command = sys.argv[1] #'GoHome' #'Start' #
	msg.Scanner_Ajustment_Angle = 0.00 #-math.pi/2  #rad -100 for no change
 	msg.Scanner_Roll_Angle = math.pi/float(sys.argv[2]) #math.pi/3       #rad -100 for no change
	msg.Scanner_Home_Angle = -100      #rad -100 for no change
	msg.Scanner_Period = float(sys.argv[3])          #sec -100 for no 
	
	#corrector2 = 1.2        
	#hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        pub.publish(msg)
        #pub2.publish(corrector2)	
        rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print('Error! Please inter 4 arguments, i.e. talker.py ~Command Pi/~Roll_angle ~Period')
    else:
        try:
            talker()
        except rospy.ROSInterruptException:
            pass
        
 
 

