#!/usr/bin/env python

import actionlib
import math
import numpy
import os
import rospy
import serial
import string
import sys
import threading
import time

from actionlib_msgs.msg import *
from geometry_msgs.msg import Vector3
from move_base_msgs.msg import *
from std_msgs.msg import Bool


wpsMaxNum= 10
wpsCurrNum= 0
wpsBuffer= numpy.zeros((2,wpsMaxNum))


def parseGPGGA(inputString):
     lat, NSside, lon, EWside = inputString.strip().split(',')[2:6]
     lat=(1-2*(NSside == "S"))*float(int(float(lat))/100)+(float(lat)%100.0)/60.0
     lon=(1-2*(NSside == "W"))*float(int(float(lon))/100)+(float(lon)%100.0)/60.0
     return (lat, lon)

def parseGPRMC(inputString,previous_track):
    speed,track = inputString.strip().split(',')[7:9]
    speed = float(speed)
#    print "speed = %f" % speed
    if(speed>0.01) :
        track = float(track) * math.pi/180.0
    else :
        track = previous_track
    return track

def updateFakeGps():
    global lat, lon, lat_rad, lon_rad, yaw
    print "[%s:] fake GPS listener initialising ..." % nodeName
#    TODO: add here the logic for recognising the serial port to which the simulated GPS receiver is connected
    fakeGpsCom = serial.Serial('/dev/ttyUSB0', 9600)
    fakeGpsCom.flushInput()
    fakeGpsCom.flushOutput()
    print "[%s:] fake GPS listener started" % nodeName
    while not rospy.is_shutdown():
        #data_raw=fakeGpsCom.readline()
        NMEAstring=string.join(fakeGpsCom.readline(),'')
        if NMEAstring.startswith('$GPGGA'):
            lat, lon = parseGPGGA(NMEAstring)
            lat_rad = lat *math.pi/180.0
            lon_rad = lon *math.pi/180.0
#            print "latitude: %.8f, longitude: %.8f" % (lat,lon)
#            print "longitude: %.8f" % lon
        elif NMEAstring.startswith('$GPRMC'):
#            print "starting yaw angle: %0.2f" % yaw
            yaw = parseGPRMC(NMEAstring,yaw)
#            print "updated yaw angle: %0.2f" % yaw
    fakeGpsCom.close()

def navigateWps():
    global wpClient, wpGoal
    global nodeName, wpThreshold
    global wpsCurrNum, targetLat, targetLon
    ackWpPub = rospy.Publisher('rover_ack_wp', Bool, queue_size=10)
    while not rospy.is_shutdown():
        if wpsCurrNum > 0:
            targetLat=wpsBuffer[0][0]
            targetLon=wpsBuffer[1][0]
            updateTarget() 
         
            wpGoal.target_pose.pose.position.x = wpX
            wpGoal.target_pose.pose.position.y = wpY
            wpGoal.target_pose.pose.orientation.w = 1.0
            wpGoal.target_pose.header.frame_id = "base_link"
            wpGoal.target_pose.header.stamp = rospy.Time.now() 
 
            if moveFlag:
	        wpClient.wait_for_server()	
                wpClient.send_goal(wpGoal)

                while (distance > wpThreshold) and not rospy.is_shutdown():
                    time.sleep(3)
                    print "[%s:] updating destination ..." % nodeName
                    updateTarget()
                    wpGoal.target_pose.pose.position.x = wpX
                    wpGoal.target_pose.pose.position.y = wpY
                    wpGoal.target_pose.pose.orientation.w = 1.0
                    wpGoal.target_pose.header.frame_id = "base_link"
                    wpGoal.target_pose.header.stamp = rospy.Time.now() 
                    wpClient.send_goal(wpGoal)
                    print "[%s:] new target sent." % nodeName
                 
                if distance < wpThreshold:
                    print "[%s:] W/P reached." % nodeName
                    ackWpPub.publish(True)
                    wpInd = 0
                    for wpInd in range(0,wpsCurrNum-2):
                        wpsBuffer[0][wpInd] = wpsBuffer[0][wpInd+1]
                        wpsBuffer[1][wpInd] = wpsBuffer[1][wpInd+1]
                    wpsBuffer[0][wpInd+1] = 0
                    wpsBuffer[1][wpInd+1] = 0
                    wpsCurrNum = wpsCurrNum - 1
                    
                else:
                    print "[%s:] navigation interrupted." % nodeName
                    return

def updateTarget():
    global ne, me, h, targetLat, targetLon, sat
    global lat_rad, lon_rad, yaw, wpX, wpY, distance
    print "[%s:] set destination latitude %.6f DEG and longitude %.6f DEG" % (nodeName, targetLat, targetLon)
    nedY=(targetLon*math.pi/180.0-lon_rad)*(ne+h)*math.cos(lat_rad)
    nedX=(targetLat*math.pi/180.0-lat_rad)*(me+h)
    print "[%s:] set destination in NED frame: x = %.3f m, y = %.3f m" % (nodeName, nedX, nedY)
    wpX=nedX * math.cos(-yaw) - nedY * math.sin(-yaw)
    wpY=-(nedX * math.sin(-yaw) + nedY * math.cos(-yaw))
    track = math.atan2(wpY,wpX)
    distance = math.sqrt(wpX*wpX + wpY*wpY)
    if (distance>sat):
	distanceSat = distance/abs(distance) * sat
	wpX = distanceSat*math.cos(track)
	wpY = distanceSat*math.sin(track)
        print "[%s:] set intermediate destination: x = %.3f m, y = %.3f m" % (nodeName, wpX, wpY)
    else:
        print "[%s:] set body frame destination: x = %.3f m, y = %.3f m" % (nodeName, wpX, wpY)

def manageWp(data):
    global nodeName, wpsMaxNum
    global wpsCurrNum, wpsBuffer
    if data.z > 0:
        wpsBuffer = numpy.zeros((2, wpsMaxNum))
        wpsCurrNum = 1
        wpsBuffer[0][wpsCurrNum] = data.x
        wpsBuffer[1][wpsCurrNum] = data.y
        print "[%s:] stored W/P number %d latitude %.6f, longitude %.6f" % (nodeName, wpsCurrNum, data.x, data.y)
    elif wpsCurrNum < wpsMaxNum:
        wpsBuffer[0][wpsCurrNum] = data.x
        wpsBuffer[1][wpsCurrNum] = data.y
        print "[%s:] stored W/P number %d latitude %.6f, longitude %.6f" % (nodeName, wpsCurrNum, data.x, data.y)
        wpsCurrNum = wpsCurrNum + 1
#    if wpsCurrNum > 0:
#        targetLat=wpsBuffer[0][0]
#        targetLon=wpsBuffer[1][0]
#        updateTarget() 
#         
#        wpGoal.target_pose.pose.position.x = wpX
#        wpGoal.target_pose.pose.position.y = wpY
#        wpGoal.target_pose.pose.orientation.w = 1.0
#        wpGoal.target_pose.header.frame_id = "base_link"
#        wpGoal.target_pose.header.stamp = rospy.Time.now() 
# 
#        if moveFlag:
#	    wpClient.wait_for_server()	
#            wpClient.send_goal(wpGoal)
#
#            while (distance > wpThreshold) and not rospy.is_shutdown():
#                print "[%s:] updating destination ..." % nodeName
#                updateTarget()
#                wpGoal.target_pose.pose.position.x = wpX
#                wpGoal.target_pose.pose.position.y = wpY
#                wpGoal.target_pose.pose.orientation.w = 1.0
#                wpGoal.target_pose.header.frame_id = "base_link"
#                wpGoal.target_pose.header.stamp = rospy.Time.now() 
#                wpClient.send_goal(wpGoal)
#                print "[%s:] new target sent." % nodeName
#                time.sleep(3)
#                
#            if distance < wpThreshold:
#                print "[%s:] W/P reached." % nodeName
#                for wpInd in range(0,wpsCurrNum-2):
#                    wpsBuffer[0][wpsInd] = wpsBuffer[0][wpsInd+1]
#                    wpsBuffer[1][wpsInd] = wpsBuffer[1][wpsInd+1]
#                wpsBuffer[0][wpsInd+1] = 0
#                wpsBuffer[1][wpsInd+1] = 0
#                wpsCurrNum = wpsCurrNum - 1
#                    
#            else:
#                print "[%s:] navigation interrupted." % nodeName
#                return

def wpsListener(argv):
    global wpClient, wpGoal
    global nodeName, moveFlag, ne, me, h, sat, wpThreshold
    global targetLat, targetLon, wpX, wpY
    global lat, lon, yaw, distance
    earth_radius=6378137
    earth_ecc=0.8181919
    sat=4.0
    wpThreshold=1
    testMode=True
    
    nodeName = os.path.basename(argv[0])
    runMode = argv[1]
    if len(argv) < 3:
        sys.stderr.write(" ERROR: please specify a valid mode as argument of this function. \n Usage: rosrun donkeyMoveToPose %s {mode [0 = body frame mode; 1 = fake GPS]} {motion [test = only destination calculation; move = also rover motion]} ({body frame x coordinate [m]} {body frame y coordinate [m]}) ({initial rover track angle [DEG]} ({target latitude [DEG]} {target longitude [DEG]})) \n Examples: \n rosrun donkeyMoveToPose %s 0 move 1 -0.5 \n rosrun donkeyMoveToPose %s 1 move 60 \n rosrun donkeyMoveToPose %s 1 move 150 44.153278 12.241426" % (os.path.basename(argv[0]),os.path.basename(argv[0]),os.path.basename(argv[0]),os.path.basename(argv[0])))
        return 2
    # TODO: complete here the management of the signature arguments errors
    
    print "[%s:] W/Ps listener initialising ..." % nodeName
    rospy.init_node('wpsListener', anonymous=True)
    wpClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    wpGoal = MoveBaseGoal()
    if argv[2] == "move":
        print "[%s:] W/Ps listener started: rover movement enabled" % nodeName
    else:
        print "[%s:] W/Ps listener started: rover movement disabled" % nodeName

    if runMode == "0":
        print "[%s:] entering mode 0: body frame coordinates." % nodeName
        wpX = float(argv[3])
        wpY = float(argv[4])
        print "[%s:] set body frame destination: x = %.3f m, y = %.3f m" % (nodeName, wpX, wpY)
    elif runMode == "1":
        print "[%s:] entering mode 1: simulated GPS position." % nodeName
        yaw=float(argv[3])*math.pi/180
        fakeGpsUpdate = threading.Thread(target=updateFakeGps, args=())
        fakeGpsUpdate.setDaemon(True)
        fakeGpsUpdate.start()
        time.sleep(1)
        print "[%s:] moving from latitude %.6f DEG and longitude %.6f DEG, heading %s DEG" % (nodeName, lat, lon, argv[3])
        me = earth_radius*(1-earth_ecc*earth_ecc)/((1-earth_ecc*earth_ecc*math.sin(lat_rad)*math.sin(lat_rad))**1.5)
        #print "me= %0.6f" % me
        ne = earth_radius/((1-earth_ecc*earth_ecc*math.sin(lat_rad)*math.sin(lat_rad))**0.5)
        #print "ne= %0.6f" % ne
        h = 0
        if len(argv) > 4:
            targetLat=float(argv[4])
            targetLon=float(argv[5])
            updateTarget()  
        else:
            print "[%s:] listening W/Ps from ROS topic rover_wp" % nodeName
            if argv[2] == "move":
                moveFlag = True
                wpsNavigator = threading.Thread(target=navigateWps, args=())
                wpsNavigator.setDaemon(True)
                wpsNavigator.start()
            else:
                moveFlag = False
            rospy.Subscriber("rover_wp",Vector3,manageWp)
            rospy.spin()
            testMode=False
          
#        updateTarget()  
#        nedY=(targetLon*math.pi/180.0-lon_rad)*(ne+h)*math.cos(lat_rad)
#        nedX=(targetLat*math.pi/180.0-lat_rad)*(me+h)
#        print "[%s:] set destination in NED frame: x = %.3f m, y = %.3f m" % (nodeName, nedX, nedY)
#        wpX=nedX * math.cos(-yaw) - nedY * math.sin(-yaw)
#        wpY=-(nedX * math.sin(-yaw) + nedY * math.cos(-yaw))
#        track = math.atan2(wpY,wpX)
#        distance = math.sqrt(wpX*wpX + wpY*wpY)
##        print "bearing %0.3f" % track
##        print "distance %0.3f" % distance
#        if(distance>sat):
#	    distanceSat = distance/abs(distance) * sat
#	    wpX = distanceSat*math.cos(bearing0)
#	    wpY = distanceSat*math.sin(bearing0)
#            print "[%s:] set intermediate destination: x = %.3f m, y = %.3f m" % (nodeName, wpX, wpY)
#        else:
#            print "[%s:] set body frame destination: x = %.3f m, y = %.3f m" % (nodeName, wpX, wpY)
    
    if testMode:
        wpGoal.target_pose.pose.position.x = wpX
        wpGoal.target_pose.pose.position.y = wpY
        wpGoal.target_pose.pose.orientation.w = 1.0
        wpGoal.target_pose.header.frame_id = "base_link"
        wpGoal.target_pose.header.stamp = rospy.Time.now() 

        if argv[2] == "move":
	    wpClient.wait_for_server()	
            wpClient.send_goal(wpGoal)
            if runMode == "0":
                wpClient.wait_for_result()
	        #print wpClient.get_result().toString
	        print wpClient.get_state()
                # TODO: add here the logic to recognize the state and result of the rover movement
            elif runMode =="1":
                while (distance > wpThreshold) and not rospy.is_shutdown():
                    print "[%s:] updating destination ..." % nodeName
                    updateTarget()
                    wpGoal.target_pose.pose.position.x = wpX
                    wpGoal.target_pose.pose.position.y = wpY
                    wpGoal.target_pose.pose.orientation.w = 1.0
                    wpGoal.target_pose.header.frame_id = "base_link"
                    wpGoal.target_pose.header.stamp = rospy.Time.now() 
                    wpClient.send_goal(wpGoal)
                    print "[%s:] new target sent." % nodeName
	            time.sleep(3)
                
                if distance < wpThreshold:
                    print "[%s:] target reached." % nodeName
                else:
                    print "[%s:] navigation interrupted." % nodeName


if __name__ == '__main__':
    try:
        wpsListener(sys.argv)
    except rospy.ROSInterruptException:
        pass
