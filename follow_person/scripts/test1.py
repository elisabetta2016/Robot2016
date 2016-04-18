#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import Vector3
import numpy
from tf.transformations import euler_from_quaternion

### Definig global variables kE and kN
kX = 0
kY = 0

def get_param(name, default):
	try:
		v = rospy.get_param(name)
		rospy.loginfo("Found parameter: %s, value: %s"%(name, str(v)))
	except KeyError:
		v = default
		rospy.logwarn("Cannot find value for parameter: %s, assigning "
				"default: %s"%(name, str(v)))
	return v

def cnst(eX,eY,Vmax,b,R):
    global kX,kY
    if eY >= 0 and eY < 0.0001:
	eY = 0.0001
    elif  eY < 0 and eY > -0.0001:
        eY = -0.0001
    ## Calculating the constraints
    if eX >= 0 and eY >= 0:
       kY = Vmax / (eX/eY+2*b/R)
    if eX >= 0 and eY <= 0:
       kY = Vmax/(eX/eY-2*b/R)
    if eX <= 0 and eY <= 0:
       kY = -Vmax/(eX/eY+2*b/R)
    if eX <= 0 and eY >= 0:
       kY = -Vmax/(eX/eY-2*b/R)
    kX = eX*kY/eY
    ## Absulute values
    kX = math.fabs(kX)
    kY = math.fabs(kY)
    return 0;

def flp():
   global kX,kY
   Vmax = 0.8

   R = 0.8
   rospy.init_node('flp', anonymous=True)
   rate = rospy.Rate(20)
   b = get_param('~distance_b', 0.4)
   Tracking_precision = get_param('~Tracking_precision', 0.02)
   CG = get_param('~Controller_Gain', 1)

   listener = tf.TransformListener()
   listener2 = tf.TransformListener()
   pub = rospy.Publisher('speedfollow', Vector3, queue_size=5)
   pub2 = rospy.Publisher('error', Vector3, queue_size=5)
   rospy.sleep(10.)
   rospy.loginfo("Following process Started, RUN !!!!")
   while not rospy.is_shutdown():
      try:
            (transRov,rotRov) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

      try:
            (transPer,rotPer) = listener2.lookupTransform('/b_link', '/head_1', rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue 
      eX = transPer[0]
      eY = transPer[1]
      if math.fabs(eX) < Tracking_precision:
          eX=0
      if math.fabs(eY) < Tracking_precision:
          eY=0
      angels = euler_from_quaternion(rotRov)
      th = angels[2]
      cnst(eX,eY,Vmax,b,R);
      Ux = math.atan(CG*eX)*kX*2/3.14159265359
      Uy = math.atan(CG*eY)*kY*2/3.14159265359
      U = numpy.matrix(( (Ux),(Uy) )).transpose()
      speed = Vector3()
      error = Vector3()
      VRL = numpy.matrix (( (1,-R/2/b),(1,R/2/b) ))*U
      speed.x = 1*VRL.item(1)
      speed.y = 1*VRL.item(0)
      speed.z = 0 #th*180/3.1415 
      pub.publish(speed)
      error.x = transPer[0]
      error.y = transPer[1]
      error.z = 0
      pub2.publish(error)
      rate.sleep()

if __name__ == '__main__':
    try:
        flp()
    except rospy.ROSInterruptException:
        pass
