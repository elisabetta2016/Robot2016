#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import Vector3
import numpy
from tf.transformations import euler_from_quaternion

def flp():
   rospy.init_node('flp', anonymous=True)
   rate = rospy.Rate(1)
   listener = tf.TransformListener()
   listener2 = tf.TransformListener()
   #pub = rospy.Publisher('speedfollow', Vector3, queue_size=5)
   while not rospy.is_shutdown():
      a = 0.2;
      #b = numpy.matrix( ((numpy.cos(a),1),(1,1)) )
      b = numpy.matrix(((1,2),(3,4)))     
      c = numpy.matrix(((1),(0)))
      c = c.transpose()
      d = b*c
      mm = d.item(1)
      try:
            (transRov,rotRov) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

      try:
            (transPer,rotPer) = listener2.lookupTransform('/base_link', '/b_link', rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue 
      eE = transPer[0] - transRov[0]
      angels = euler_from_quaternion(rotRov)
      print angels
      speed = Vector3()
      speed.x = d.item(0)
      speed.y = d.item(1)
      speed.z = 0;
      pub.publish(speed)
      rate.sleep()

if __name__ == '__main__':
    try:
        flp()
    except rospy.ROSInterruptException:
        pass
