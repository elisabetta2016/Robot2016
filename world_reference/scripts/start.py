#!/usr/bin/env python
import roslib
import rospy
import sys
import math
import time
import tf
import os
from custom_msgs.msg import positionEstimate
from custom_msgs.msg import orientationEstimate
from nav_msgs.msg import Odometry

#PyKML
from pykml.factory import KML_ElementMaker as KML
from lxml import etree

carryX = 0.0
carryY = 0.0
carryYaw = 0.0
initLat = 0.0
filebool = False
initBool =True
firstPose = True
lat = 0.0
initLon = 0.0
h = 0
earth_radius=6378137
earth_ecc=0.8181919
me=0.0
ne=0.0
lat_rad = 0
lon_rad = 0
count = 0
angles = [0,0,0]
a = 0
debug = False
bear_map_world = 0.0
br = tf.TransformBroadcaster()
listener = tf.TransformListener()

def filt(x, y, a) :
    global firstPose
    if(vt == 0 and firstPose == True) :
        a = 1
    elif(vt == 0 and firstPose == False) :
        a = 1
    elif(vt > 0.1) : 
        a = 0.9999
        firstPose = False
    
    y = (1 - a) * x + a * y
    return y


def check(argv) :
    global fld, name_object, filebool, debug

    #fld = KML.Folder()
    if(len(argv) > 2) :
       sys.stderr.write("usage:  ")
    
    elif(len(argv) == 2) : 
       if(argv[1] != "log" and argv[1] != "debug") :
           sys.stderr.write("the argument must be log or debug\n")
           sys.exit(2)
       if(argv[1] == "log") :  
           filebool = True
       if(argv[1] == "debug") :
           debug = True
       
       #name_object = KML.name("KML_Document")
       #filename = tkFileDialog.asksaveasfilename(**self.file_opt)
       
    

def fileClose() :
    global outfile, fld
    outfile = file(__file__.rstrip('.py')+'.kml', 'w')
    outfile.write(etree.tostring(fld, pretty_print=True))
    #filename.write(etree.tostring(fld, pretty_print=True))
    #filename.close()


def callback1(data) :
    global initBool, initLat, initLon, me, ne, lat, lon, carryY, carryX, fld, filebool, angles, bear_map_world, lat_rad, lon_rad
    if initBool:    
        lat=data.latitude
        lon=data.longitude
        initBool = False
    else:
        lat = filt(data.latitude, lat, a)
        lon = filt(data.longitude, lon, a)
    lat_rad = lat *math.pi/180.0
    lon_rad = lon *math.pi/180.0
    #print lat, lon



'''    if(filebool == True) :
        pm = KML.Placemark(KML.name("Placemark %d" % count), KML.Point(KML.coordinates("%f,%f" % (lat, lon))))
        fld.append(pm)
        count = count + 1
    if(initLat == 0) :
        try :
            br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, carryYaw, "sxyz"), rospy.Time.now(), 'base_link', 'dummy')
            (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time())
            angles = tf.transformations.euler_from_quaternion(rot)
            bear_map_world = -(carryYaw + angles[2])
            initLat = lat_rad
            initLon = lon_rad
            me = earth_radius*(1-earth_ecc*earth_ecc)/((1-earth_ecc*earth_ecc*math.sin(lat_rad)*math.sin(lat_rad))**1.5)
            ne = earth_radius/((1-earth_ecc*earth_ecc*math.sin(lat_rad)*math.sin(lat_rad))**0.5)
            br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, bear_map_world), rospy.Time.now(), 'map', 'world')
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException),err:
            print str(err)#"error"
    else :
         try:
             carryY=(lon_rad - initLon)*(ne+h)*math.cos(lat_rad)
             carryX=(lat_rad - initLat)*(me+h)
             #print carryYaw
             #print carryX
             #print carryY
             br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, bear_map_world), rospy.Time.now(), 'map', 'world')
             br.sendTransform((-carryX, -carryY, 0), tf.transformations.quaternion_from_euler(0, 0, 0, "sxyz"), rospy.Time.now(), 'base_link', 'dummy')
         except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException),err:
            print str(err)#"error"
'''


def debugging(latitude, longitude) :
    global initLat, initLon, me, ne, lat, lon, carryY, carryX, fld, filebool, angles
    lat=latitude
    lon=longitude
    #lat=filt(lat, a)
    #lon=filt(lon, a)
    #print lat, lon
    lat_rad = lat *math.pi/180.0
    lon_rad = lon *math.pi/180.0
    if(filebool == True) :
        pm = KML.Placemark(KML.name("Placemark %d" % count), KML.Point(KML.coordinates("%f,%f" % (lat, lon))))
        fld.append(pm)
        count = count + 1
    try :
        #print angles[2]
        if(initLat == 0) :
            br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, carryYaw, "sxyz"), rospy.Time.now(), 'base_link', 'dummy')
            (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time())
            angles = tf.transformations.euler_from_quaternion(rot)
            initLat = lat_rad
            initLon = lon_rad
            me = earth_radius*(1-earth_ecc*earth_ecc)/((1-earth_ecc*earth_ecc*math.sin(lat_rad)*math.sin(lat_rad))**1.5)
            ne = earth_radius/((1-earth_ecc*earth_ecc*math.sin(lat_rad)*math.sin(lat_rad))**0.5)
            br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, -(carryYaw + angles[2])), rospy.Time.now(), 'map', 'world')
            #br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, carryYaw), rospy.Time.now(), 'world', 'map')
        else :
            carryY=(lon_rad - initLon)*(ne+h)*math.cos(lat_rad)
            carryX=(lat_rad - initLat)*(me+h)
            print carryYaw
            #br.sendTransform((carryX -trans[0], carryY -trans[1], 0), tf.transformations.quaternion_from_euler(0, 0, carryYaw -angles[2], "sxyz"), rospy.Time.now(), 'base_link', 'world')
            br.sendTransform((-carryX, -carryY, 0), tf.transformations.quaternion_from_euler(0, 0, carryYaw, "sxyz"), rospy.Time.now(), 'base_link', 'dummy')
    except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) :
        print "exception"
    

    
def callback2(data) :
   global carryYaw
   carryYaw = data.yaw *math.pi/180.0
   #print carryYaw
#   br.sendTransform((carryX, carryY, 0), tf.transformations.quaternion_from_euler(0, 0, carryYaw -angles[2], "sxyz"), rospy.Time.now(), 'map', 'world')

# - Explanation of last string argument of tf.transformations.quaternion_from_euler
#A triple of euler angles can be applied/interpreted in 24 ways, which can be specified using a 4 character string or encoded 4-tuple:
# *Axes 4-string: e.g. 'sxyz' or 'ryxy'
# - first character: rotation are applied to 's'tatic or 'r'otating frame
# - remaining characters : successive rotation axis 'x', 'y' or 'z'

def updateVelocity(data) :
    global vx, vy, vt
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    vt = math.sqrt(vx*vx + vy*vy)
    #print "vel: %f" % vt
    
def daemonize() :
    #global lat_rad, lon_rad
    x1 = 44.491825
    x2 = 11.330118
    try :
       pid = os.fork()
       print pid
    except OSError, e:
        raise Exception, "la prima fork e fallita"

    if(pid == 0) :
        os.setsid()
        try :
            pid = os.fork()
            print pid
        except OSError, e:
            raise Exception, "la seconda fork e fallita"
        if(pid == 0) :
            global lat_rad, lon_rad
            while True :
                x1 = x1 + 0.000001
                x2 = x2 + 0.000001
                lat_rad = x1 * math.pi/180
                lon_rad = x2 * math.pi/180
                print "demone", lat_rad, lon_rad
                time.sleep(2)
            os.exit(0)

if __name__ == '__main__':
    rospy.init_node('world_reference')
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    check(sys.argv)
    rate = rospy.Rate(3) #5Hz
    if(debug == True) :
        daemonize()

    while not rospy.is_shutdown() :
        #for timeStep in range(init_time*update_rate):
        #br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0, "sxyz"), rospy.Time.now(), 'base_link', 'dummy')
        rate.sleep()
        rospy.Subscriber("mti/filter/orientation",orientationEstimate,callback2)
        rospy.Subscriber("/odom", Odometry, updateVelocity)
        #print "prova:", lat_rad

        if(debug == False) :    
            rospy.Subscriber("mti/filter/position",positionEstimate,callback1)
        

        if(filebool == True) :
            pm = KML.Placemark(KML.name("Placemark %d" % count), KML.Point(KML.coordinates("%f,%f" % (lat, lon))))
            fld.append(pm)
            count = count + 1
        if(initLat == 0) :
            try :
                br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, carryYaw, "sxyz"), rospy.Time.now(), 'base_link', 'dummy')
                (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time())
                angles = tf.transformations.euler_from_quaternion(rot)
                bear_map_world = -(carryYaw + angles[2])
                initLat = lat_rad
                initLon = lon_rad
                print "latrad lonrad ", lat_rad, lon_rad
                me = earth_radius*(1-earth_ecc*earth_ecc)/((1-earth_ecc*earth_ecc*math.sin(lat_rad)*math.sin(lat_rad))**1.5)
                ne = earth_radius/((1-earth_ecc*earth_ecc*math.sin(lat_rad)*math.sin(lat_rad))**0.5)
                br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, bear_map_world), rospy.Time.now(), 'map', 'world')
            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException),err:
                print str(err)#"error"
        else :
             try:
                 carryY = (lon_rad - initLon)*(ne+h)*math.cos(lat_rad)
                 carryX = (lat_rad - initLat)*(me+h)
                 print "Yaw", carryYaw
                 print "carryX",carryX
                 print "carryY", carryY
                 print "lat %f  lon %f" % (lat, lon)
                 print "initlat initoLon",initLat, initLon
                 br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, bear_map_world), rospy.Time.now(), 'map', 'world')
                 br.sendTransform((-carryX, -carryY, 0), tf.transformations.quaternion_from_euler(0, 0, 0, "sxyz"), rospy.Time.now(), 'base_link', 'dummy')
             except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException),err:
                 print str(err)#"error"

            #if(filebool == True) :
            #    fileClose()

    
