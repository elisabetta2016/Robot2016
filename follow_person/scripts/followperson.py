#!/usr/bin/env python
import rospy
import math
import tf
import numpy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3

### Definig global variables kE and kN
kE = 0
kN = 0



def cnst(eN,eE,Vmax,b,R):
    global kE,kN
    if eE >= 0 and eE < 0.0001:
	eE = 0.0001
    elif  eE < 0 and eE > -0.0001:
        eE = -0.0001
    ## Calculating the constraints
    if eN >= 0 and eE >= 0:
       kE = Vmax / (eN/eE+2*b/R)
    if eN >= 0 and eE <= 0:
       kE = Vmax/(eN/eE-2*b/R)
    if eN <= 0 and eE <= 0:
       kE = -Vmax/(eN/eE+2*b/R)
    if eN <= 0 and eE >= 0:
       kE = -Vmax/(eN/eE-2*b/R)
    kN = eN*kE/eE
    ## Absulute values
    kN = math.fabs(kN)
    kE = math.fabs(kE)
    return 0;

def followperson():
    global kE,kN
    Vmax = 1
    b = 0.4
    R = 0.8
    listener = tf.TransformListener()
    listener2 = tf.TransformListener()
    pub = rospy.Publisher('speedfollow', Vector3, queue_size=5)
    rospy.init_node('followperson', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    #Creating the errors
    
    while not rospy.is_shutdown():
       try:
            (transRov,rotRov) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
       except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

       try:
            (transPer,rotPer) = listener2.lookupTransform('/b_link', '/head_1', rospy.Time(0))
       except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue 
       eE = transPer[0] - transRov[0]
	   
       eN = transPer[1] - transRov[1]
       angels = euler_from_quaternion(rotRov)
       th = angles[2]
       cnst(eN,eE,Vmax,b,R);
       Un = math.atan(10*eN)*kN
       Ue = math.atan(10*eE)*kE
       VRL = numpy.matrix(( (numpy.cos(th)+numpy.sin(th),numpy.sin(th)-numpy.cos(th)),(numpy.cos(th)-numpy.sin(th),numpy.sin(th)+numpy.cos(th) ))
       
       s = Vector3() 
       s.x = VRL.item(0)
       s.y = VRL.item(1)
       s.z = 0.0       
       pub.publish(s)
       rate.sleep()

if __name__ == '__main__':
    try:
        flp()
    except rospy.ROSInterruptException:
        pass        
	
	
	
