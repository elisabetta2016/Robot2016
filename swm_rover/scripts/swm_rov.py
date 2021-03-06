#!/usr/bin/env python

#****************************************************************************
#
# Sherpa World Model Interface
#
# This software rover developed at:
# Casy Lab- Bologna
#
# Description:
#  Interface node with SWM
#	 - Input: name of the rover
#
# Authors:
# Mohsen <seyedmohs.mirhassani@unibo.it>
#
# Created in 05/02/2016.
#
#
# Copyright (C) 2015 PRISMA Lab. All rights reserved.
#****************************************************************************/


#---Ros lib
import rospy
import roslib
import actionlib
#---

import os.path
import ConfigParser
import zmq
import random
import sys
import time
import json

import sys
import time
import re
import os
import thread
import multiprocessing
sys.path.append('/home/sherpa/DCM/ubx_robotscenegraph/examples/sherpa/mission_scripts')
import swm


#from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import Joy
from sherpa_msgs.msg import GeoPoint

from pyutil.fipa import *
from pyutil.wdbutil import *
from pyutil.consutil import *

#-Receive the position of # rover form SWM 				-> publish on Ros Topic
class swm_interface_rover( object ):

	def __init__( self, name ):
		self.rover_name = name

	def getgenID(self, genius_query):
		context = zmq.Context()
		socket = context.socket(zmq.REQ)
		socket.connect("tcp://localhost:22422")		
		socket.send_string(json.dumps(genius_query))
		queryResult = socket.recv_json()
		Result = queryResult['ids']
		Result = Result[0]
		return Result

	def getoriginID(self):
		query = {"@worldmodeltype": "RSGQuery","query": "GET_NODES","attributes": [{"key": "gis:origin", "value": "wgs84"},]}
		context = zmq.Context()
		socket = context.socket(zmq.REQ)
		socket.connect("tcp://localhost:22422")		
		socket.send_string(json.dumps(query))
		queryResult = socket.recv_json()
		Result = queryResult['ids']
		Result = Result[0]
		return Result


	def get_geopose(self, node_id, origin_id):
		context = zmq.Context()
		socket = context.socket(zmq.REQ)
		socket.connect("tcp://localhost:22422")		
		p = GeoPoint()
		attributes_query =  ({ "@worldmodeltype": "RSGQuery", "query": "GET_TRANSFORM", "id": node_id, "idReferenceNode": origin_id})
		socket.send_string(json.dumps(attributes_query))
		result = socket.recv_json()
		result = result['transform']['matrix']
		#position		
		p.latitude = float(result[0][3])
		p.longitude = float(result[1][3])
		p.altitude = float(result[2][3])

		return p
		


	
	def run(self):

		rate = rospy.Rate(1) # 10hz
		#---Rover 0
		genius_pub = rospy.Publisher('/leashing_target_position', GeoPoint, queue_size=1)		

		#genius_query = { "@worldmodeltype": "RSGQuery","query": "GET_NODES","attributes": [{"key": "name", "value": "genius_geopose"},]}
		#node_id = self.getgenID(genius_query)
		#origin_id = self.getoriginID()
		#print origin_id
		while not rospy.is_shutdown():
			result = swm.getGeopose("genius")
			print result
			#p = self.get_geopose(node_id,origin_id)
			genius_pub.publish( p )
			rate.sleep()
				

if __name__ == '__main__':

	rover_name = "rover_" #+ sys.argv[1]
	rospy.init_node('CREATE_SWM_' + rover_name)
	interface = swm_interface_rover( rover_name )
	interface.run()


