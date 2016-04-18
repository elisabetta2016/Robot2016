#!/usr/bin/env python


import rospy
import os.path
import serial
import string
import sys
import threading
import math
 
#LOS
import time
from Los.Client import Connection
from Los.Types import *
from Los.Util import *
import actionlib
from move_base_msgs.msg import *
from actionlib_msgs.msg import *

#GPS
from custom_msgs.msg import gnssSample
from custom_msgs.msg import positionEstimate 
from custom_msgs.msg import orientationEstimate

threshold = 120 * math.pi/180
earth_radius=6378137
earth_ecc=0.8181919
lat=0.0
targetLat=0.0
targetLon=0.0
lon=0.0
#fix=0.0
h=0.0
yaw =0.0
distance=0.0
flagMotion=False
fakeGpsOn=False
x=0
y=0
sat=4.0
bearing=0.0

def callback(data):
    global lat,lon,fix,h 
    
    #print "callBack: latitude: %0.7f longitude: %0.7f h: %0.3f\n" % (data.latitude,data.longitude,data.hEll)    
    lat=data.latitude   
    lon=data.longitude
    h=data.hEll 
    #fix=data.fix

def callbackEstimate(data):
     global yaw
     yaw= data.yaw
     yaw = 180.0*math.pi/180.0
#     print "updated yaw: %0.2f" % yaw

def parseGPGGA(inputString):
     lat, NSside, lon, EWside = inputString.strip().split(',')[2:6]
     lat=(1-2*(NSside == "S"))*float(int(float(lat))/100)+(float(lat)%100.0)/60.0
     lon=(1-2*(NSside == "W"))*float(int(float(lon))/100)+(float(lon)%100.0)/60.0
     return (lat, lon)

def parseGPRMC(inputString,previous_track):
    speed,track = inputString.strip().split(',')[7:9]
    speed = float(speed)
    if(speed>0.01) :
        track = float(track)
    else :
        track = previous_track
    return track

def updateFakeGps():
     global lat,lon,lat_rad,lon_rad,fakeGpsOn,yaw
     print "fake GPS reading starting ..."
     ser = serial.Serial('/dev/ttyUSB1', 9600)
     ser.flushInput()
     ser.flushOutput()
     print "fake GPS reading succesfully started!"
     while True:
         data_raw=ser.readline()
         NMEAstring=string.join(data_raw,'')
         if NMEAstring.startswith('$GPGGA'):
             lat, lon = parseGPGGA(NMEAstring)
             lat_rad = lat *math.pi/180.0
             lon_rad = lon *math.pi/180.0
#             print "latitude: %.8f" % lat
#             print "longitude: %.8f" % lon
         elif NMEAstring.startswith('$GPRMC'):
             yaw = parseGPRMC(NMEAstring,yaw) *math.pi/180.0
#             print "track angle: %.2f" % track
             print "updated yaw: %0.2f" % yaw
     ser.close()

def updateTargetFake():
    global lon_rad,lat_rad,h,yaw,distance
    targetUpdateFlag=True
    print "distance %.1f"%distance
    testInd=0
    while(distance>1.0):#(targetUpdateFlag):
        testInd=testInd+1
        print "distance while %.1f"%distance
    	print "target recalculation"
        me=earth_radius*(1-earth_ecc*earth_ecc)/((1-earth_ecc*earth_ecc*math.sin(lat_rad)*math.sin(lat_rad))**1.5)
        ne=earth_radius/((1-earth_ecc*earth_ecc*math.sin(lat_rad)*math.sin(lat_rad))**0.5)
        y1=(targetLon*math.pi/180.0-lon_rad)*(ne+h)*math.cos(lat_rad)
        x1=(targetLat*math.pi/180.0-lat_rad)*(me+h)
        print "x1=%0.3f"%x1
        print "y1=%0.3f"%y1
        print "yaw=%.2f"%yaw
        x=x1*math.cos(-yaw) - y1*math.sin(-yaw)
        y=x1*math.sin(-yaw) + y1*math.cos(-yaw)

        theta=math.atan2(y,x)
        bearing0=math.atan2(y,x)
        distance = math.sqrt(x*x + y*y)
        print "distance %0.3f" % distance
        print "bearing %0.3f" % bearing0

        if(distance>sat):
	    distanceSat = distance/abs(distance) * sat
	    x= distanceSat*math.cos(bearing0)
	    y= distanceSat*math.sin(bearing0)
#        else:
#            targetUpdateFlag=False

        print "CALCOLO ESEGUITO x: %f y: %f \n" % (x,y)

#        print "Drive here in simulated GPS mode"
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	goal = MoveBaseGoal()

   	print "RIDOTTO x: %f y: %f" % (x,y)	
	goal.target_pose.pose.position.x = x
 	goal.target_pose.pose.position.y = - y
	goal.target_pose.pose.orientation.w=1.0
	goal.target_pose.header.frame_id= "base_link"
	goal.target_pose.header.stamp=rospy.Time.now()
        print "Got Here" 
	#client.wait_for_server()	
	client.send_goal(goal)
        print "Also Here" 
        print "distance while end %.1f"%distance
			
#	client.wait_for_result()
		
#        print client.get_result()
        time.sleep(1)


def updateTarget():
	global distance, flagMotion,x,y,theta,bearing,bearing0,targetLat,targetLon,targetUpdateFlag

        print flagMotion
	print distance
		
	print "yaw=%0.2f"%yaw

	while (flagMotion & (distance > 2)) : 
		time.sleep(3)
		#ricalcolo destinazione :
    		print "\ndentro il while:"
		print "lat=%0.7f lon=%0.7f h=%0.3f" % (lat,lon,h)
     		me=(6378137 * (1-math.exp(2)))/(abs((1-(math.exp(2)*(math.sin(lat* math.pi/180.0)*math.sin(lat* math.pi/180.0)))))**(1.5))     		
     		ne=(6378137)/((abs(1-(math.exp(2)*(math.sin(lat*math.pi/180.0))*math.sin(lat* math.pi/180.0))))**(0.5))
     		print me
		print ne
                print targetLat
		print targetLon
     		y1=((targetLon* math.pi/180.0)-(lon* math.pi/180.0))*(ne+h)*math.cos(lat* math.pi/180.0)
     		x1=((targetLat* math.pi/180.0)-(lat* math.pi/180.0))*(me+h)
     		#theta=float(argv[5])
		print "yaw=%0.2f"%yaw
   		print "x1=%0.3f"%x1
		print "y1=%0.3f"%y1
		print "distance=%0.1f"%distance
    		x=x1*math.cos(-yaw) - y1*math.sin(-yaw)
     		y=x1*math.sin(-yaw) + y1*math.cos(-yaw)

     		print "CALCOLO ESEGUITO x: %f y: %f" % (x,y)		
		#getDistance modulo:
		distance = math.sqrt(x*x + y*y)
		print "distance %0.3f" % distance    	
		bearing = math.atan2(y,x) - bearing0
		print "bearing %0.3f\n" % bearing 

		if(distance>sat):
			distanceSat = distance/abs(distance) * sat
			x= distanceSat*math.cos(math.atan2(y,x))
			y= distanceSat*math.sin(math.atan2(y,x))
	

		#getBearingToDestination atan(y/x) - yaw :
        	

		if(bearing>math.pi)	:
			bearing= 2*math.pi - bearing
			print "bearing %0.3f\n" % bearing 
		 
		bearing = abs(bearing)
	
    		print "RIDOTTO x: %f y: %f" % (x,y)	
	
		if( (distance>2)) :#(bearing > threshold) |
			print "target Update"
			bearing0=math.atan2(y,x)
			client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
			goal = MoveBaseGoal()

			goal.target_pose.pose.position.x = x
 			goal.target_pose.pose.position.y =y
			goal.target_pose.pose.orientation.w=1.0
			goal.target_pose.header.frame_id= "base_link"
			goal.target_pose.header.stamp=rospy.Time.now() 
			client.wait_for_server()	
			client.send_goal(goal)
			
			client.wait_for_result()
		
			print client.get_result()
			print client.get_state()
			
		

def moveTo(argv):  
    global lat_rad,lon_rad,h,yaw
    global fakeGpsOn,targetLat,targetLon
    global distance, flagMotion,x,y,theta,bearing,bearing0,sat

# #************************check the arguments*********************
    if len(argv) < 4:
        sys.stderr.write("Usage: donkeyMoveToPose %s {mode} [0 = local cartesian frame;1 = fake GPS] {Localization.active} [bool] {X} [m] {Y} [m] {Theta} [rad] \n ex.:\n rosrun donkeyMoveToPose %s False False 0.5 0.0 0.5 \n\n rosrun donkeyMoveToPose %s True False 49.56789 11.45677 \n\n" % (os.path.basename(argv[0]),os.path.basename(argv[0]),os.path.basename(argv[0])))
        return 2
# TODO: da perfezionare la logica di errore riportata sotto e quella sopra
#    if(argv[1]!="True" and argv[1]!="False"):
#        sys.stderr.write("The 1th argument must be False or True \n\n")
#        return 3
#    if(argv[2]!="True" and argv[2]!="False"):
#        sys.stderr.write("The 2th argument must be False or True \n\n")
#        return 3

# #*****************connection to ANT************************

    rospy.init_node('donkeyGPSListener', anonymous=True)
#    rospy.Subscriber("mti/filter/orientation",orientationEstimate,callbackEstimate)

    proxy = Connection(("192.168.8.149", 1234), timeout=2.5)
    proxy.open()
    proxy.ping()
    proxy.login("User", "none")
    print "ANT is connencted."

    if(argv[1]=="0") :
# #********************MOVE TO (WP in the map)*************************
	
        rospy.init_node('donkeyGPSListener', anonymous=True)
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	goal = MoveBaseGoal()

	wp_num = (len(argv)-3)/3
	print "goal: x=%0.2f y=%0.2f" % (float(argv[3]),float(argv[4]))
	goal.target_pose.pose.position.x = float(argv[3])
 	goal.target_pose.pose.position.y = float(argv[4])
	goal.target_pose.pose.orientation.w=1.0
	goal.target_pose.header.frame_id= "map"
	print "frame_id= map"
	goal.target_pose.header.stamp=rospy.Time.now() 
	client.wait_for_server()	
	client.send_goal(goal)
	client.wait_for_result()
		
	print "result:"
	print client.get_result()
	print "state:"
	print client.get_state()
	
	wp_num=wp_num -1


	while(wp_num!=0) : 
		

	     if(client.get_state()==3):
		
		goal.target_pose.pose.position.x = float(argv[3+3*wp_num])
 		goal.target_pose.pose.position.y = float(argv[4+3*wp_num])
		goal.target_pose.pose.orientation.w=1.0
		goal.target_pose.header.frame_id= "map"
		goal.target_pose.header.stamp=rospy.Time.now() 
	
		client.wait_for_server()	

		client.send_goal(goal)
		client.wait_for_result()
		
		print client.get_result()
		wp_num=wp_num -1

	     else :
		print client.get_state()

    elif (argv[1]=="1") :
        
        rospy.init_node('donkeyGPSListener', anonymous=True)
        #rospy.Subscriber("mti/filter/orientation",orientationEstimate,callbackEstimate)

	#Thread
        fakeGpsOn=True
	fakeGpsUpdate = threading.Thread(target=updateFakeGps, args=())
	fakeGpsUpdate.setDaemon(True)
	fakeGpsUpdate.start()
        targetUpdateFlag=True
	targetUpdate = threading.Thread(target=updateTargetFake, args=())
	targetUpdate.setDaemon(True)
        time.sleep(1)

        print "moving from latitude %.6f and longitude %.6f" % (lat,lon)
        h=0
        yaw=float(argv[6])*math.pi/180
        me=earth_radius*(1-earth_ecc*earth_ecc)/((1-earth_ecc*earth_ecc*math.sin(lat_rad)*math.sin(lat_rad))**1.5)
        print "me= %0.6f" % me
        ne=earth_radius/((1-earth_ecc*earth_ecc*math.sin(lat_rad)*math.sin(lat_rad))**0.5)
        print "ne= %0.6f" % ne
        targetLat=float(argv[3])
        targetLon=float(argv[4])
#        print "%.6f"%(targetLon*math.pi/180.0-lon_rad)
#        print "%.6f"%(targetLat*math.pi/180.0-lat_rad)
        y1=(targetLon*math.pi/180.0-lon_rad)*(ne+h)*math.cos(lat_rad)
        x1=(targetLat*math.pi/180.0-lat_rad)*(me+h)
        print "x1=%0.3f"%x1
        print "y1=%0.3f"%y1
        print "yaw=%.2f"%yaw
        x=x1*math.cos(-yaw) - y1*math.sin(-yaw)
        y=x1*math.sin(-yaw) + y1*math.cos(-yaw)

        theta=math.atan2(y,x)
        bearing0=math.atan2(y,x)
        distance = math.sqrt(x*x + y*y)
        print "distance %0.3f" % distance
        print "bearing %0.3f" % bearing0

        if(distance>sat):
	    distanceSat = distance/abs(distance) * sat
	    x= distanceSat*math.cos(bearing0)
	    y= distanceSat*math.sin(bearing0)
        else:
            targetUpdateFlag=False
        print "CALCOLO ESEGUITO x: %f y: %f \n" % (x,y)

#        print "Drive here in simulated GPS mode"
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	goal = MoveBaseGoal()

   	print "RIDOTTO x: %f y: %f" % (x,y)	
	goal.target_pose.pose.position.x = x
 	goal.target_pose.pose.position.y = - y
	goal.target_pose.pose.orientation.w=1.0
	goal.target_pose.header.frame_id= "base_link"
	goal.target_pose.header.stamp=rospy.Time.now() 
	client.wait_for_server()	
	client.send_goal(goal)
        time.sleep(1)
	#targetUpdate.start()
	#client.wait_for_result()
		
	#print client.get_result()
	#print client.get_state()
        #time.sleep(10)
        
        #targetUpdateFlag=True
        while (distance>1):
    	    print "target recalculation"
            me=earth_radius*(1-earth_ecc*earth_ecc)/((1-earth_ecc*earth_ecc*math.sin(lat_rad)*math.sin(lat_rad))**1.5)
            ne=earth_radius/((1-earth_ecc*earth_ecc*math.sin(lat_rad)*math.sin(lat_rad))**0.5)
            y1=(targetLon*math.pi/180.0-lon_rad)*(ne+h)*math.cos(lat_rad)
            x1=(targetLat*math.pi/180.0-lat_rad)*(me+h)
            print "x1=%0.3f"%x1
            print "y1=%0.3f"%y1
            print "yaw=%.2f"%yaw
            x=x1*math.cos(-yaw) - y1*math.sin(-yaw)
            y=x1*math.sin(-yaw) + y1*math.cos(-yaw)

            theta=math.atan2(y,x)
            bearing0=math.atan2(y,x)
            distance = math.sqrt(x*x + y*y)
            print "distance %0.3f" % distance
            print "bearing %0.3f" % bearing0

            if(distance>sat):
	       distanceSat = distance/abs(distance) * sat
	       x= distanceSat*math.cos(bearing0)
	       y= distanceSat*math.sin(bearing0)
#        else:
#            targetUpdateFlag=False

            print "CALCOLO ESEGUITO x: %f y: %f \n" % (x,y)

#        print "Drive here in simulated GPS mode"
            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	    goal = MoveBaseGoal()

   	    print "RIDOTTO x: %f y: %f" % (x,y)	
	    goal.target_pose.pose.position.x = x
 	    goal.target_pose.pose.position.y = - y
	    goal.target_pose.pose.orientation.w=1.0
	    goal.target_pose.header.frame_id= "base_link"
	    goal.target_pose.header.stamp=rospy.Time.now() 
	    #client.wait_for_server()	
	    client.send_goal(goal)
			
	    #client.wait_for_result()
		
            #print client.get_result()
            time.sleep(3)

            fakeGpsOn=False
    
    elif (argv[1]=="2") :
 #******************************MOVE TO (GPS)*****************************************

     rospy.init_node('donkeyGPSListener', anonymous=True)
     #rospy.Subscriber("mti/sensor/gnssPvt", gnssSample, callback)
     rospy.Subscriber("mti/filter/orientation",orientationEstimate,callbackEstimate)
     rospy.Subscriber("mti/filter/position",positionEstimate,callback)
     time.sleep(4)
    
    #calcolo le coordinate locali:
     
     print "\nprima posizione:\nlat=%0.7f lon=%0.7f h=%0.3f\n" % (lat,lon,h)
     me=(6378137 * (1-math.exp(2)))/(1-(math.exp(2)*(math.sin(lat* math.pi/180.0)*math.sin(lat* math.pi/180.0))))**(1.5)
     print "me= %0.6f\n" % me
     ne=(6378137)/((abs(1-(math.exp(2)*(math.sin(lat*math.pi/180.0))*math.sin(lat* math.pi/180.0))))**(0.5))
     print "ne= %0.6f\n" % ne
     targetLat=float(argv[3])
     targetLon=float(argv[4])
     y1=((float(argv[4])* math.pi/180.0)-(lon* math.pi/180.0))*(ne+h)*math.cos(lat* math.pi/180.0)
     x1=((float(argv[3])* math.pi/180.0)-(lat* math.pi/180.0))*(me+h)
     print "x1=%0.3f"%x1
     print "y1=%0.3f"%y1
     print "yaw=%.2f"%yaw
     x=x1*math.cos(-yaw) - y1*math.sin(-yaw)
     y=x1*math.sin(-yaw) + y1*math.cos(-yaw)

     theta=math.atan2(y,x)
     bearing0=math.atan2(y,x)
     distance = math.sqrt(x*x + y*y)
     print "distance %0.3f" % distance
     print "bearing %0.3f" % bearing0

     if(distance>sat):
		distanceSat = distance/abs(distance) * sat
		x= distanceSat*math.cos(bearing0)
		y= distanceSat*math.sin(bearing0)
        
     print "CALCOLO ESEGUITO x: %f y: %f \n" % (x,y)
        
     try :
	
	

	rospy.init_node('donkeyGPSListener', anonymous=True)
	#Thread
	targetUpdate = threading.Thread(target=updateTarget, args=())
	targetUpdate.setDaemon(True)


        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	goal = MoveBaseGoal()

   	print "RIDOTTO x: %f y: %f" % (x,y)	
	goal.target_pose.pose.position.x = x
 	goal.target_pose.pose.position.y =y
	goal.target_pose.pose.orientation.w=1.0
	goal.target_pose.header.frame_id= "base_link"
	goal.target_pose.header.stamp=rospy.Time.now() 
	client.wait_for_server()	
	client.send_goal(goal)
	flagMotion=True	
	targetUpdate.start()
	print "thread started"
	client.wait_for_result()
		
	print client.get_result()
	print client.get_state()
	
	flagMotion= False

      #  proxy.configure(Struct({"ObstacleAvoidance.syncActive":True}))
      #  if(argv[2]=="False"):
      #      proxy.configure(Struct({"Localization.active": False}))
      #  elif(argv[2]=='True'):
      #      proxy.configure(Struct({"Localization.active":True}))
        

      #  proxy.Localization.snapToPose(0.0,0.0,yaw)	
	
	
      #	proxy.Motion.moveToPose(Float64(x),Float64(y),Float64(theta))
	


     except Exception, e:
        print str(e)
      
     finally :
        try:
            proxy.configure(Struct({"Localization.active":True}))
        except Exception, e: 
            print str(e)


    proxy.close();
   	
if __name__ == '__main__':
    try:
        moveTo(sys.argv)
    except rospy.ROSInterruptException:
        pass
