#!/usr/bin/env python

import rospy
import os.path
import sys
import serial
import string
from xbee import XBee
import math

def parseGPGGA(inputString):
     lat, NSside, lon, EWside = inputString.strip().split(',')[2:6]
     lat=(1-2*(NSside == "S"))*float(lat)/100
     lon=(1-2*(NSside == "W"))*float(lon)/100
     return (lat, lon)

def parseGPRMC(inputString):
    speed,track = inputString.strip().split(',')[7:9]
    speed = float(speed)
#    print "speed: %.2f" % speed
    track = (speed>0.01)*float(track)
    return track

def listener(argv):
    rospy.init_node('donkeyGPSListener', anonymous=True)

    ser = serial.Serial('/dev/ttyUSB1', 9600)
    #xbee = XBee(ser)
    ser.flushInput()
    ser.flushOutput()
    print("inizio lettura")
    while True:
        data_raw=ser.readline()
	NMEAstring=string.join(data_raw,'')
        if NMEAstring.startswith('$GPGGA'):
            lat, lon = parseGPGGA(NMEAstring)
#            lat, NSside, lon, EWside = NMEAstring.strip().split(',')[2:6]
#            lat=(1-2*(NSside == "S"))*float(lat)/100
#            lon=(1-2*(NSside == "W"))*float(lon)/100
            print "latitude: %.8f" % lat
            print "longitude: %.8f" % lon
        elif NMEAstring.startswith('$GPRMC'):
            track = parseGPRMC(NMEAstring)
#            speed,track = NMEAstring.strip().split(',')[7:9]
#            speed = float(speed)
#            print "speed: %.2f" % speed
#            track = (speed>0.01)*float(track)
            print "track angle: %.2f" % track
    ser.close()

if __name__ == '__main__':
    try:
        listener(sys.argv)
    except rospy.ROSInterruptException:
        pass
				
