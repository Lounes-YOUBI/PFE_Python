#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import os
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
import subprocess
import sys
from threading import Thread
 #import thread
import threading
import cv2
import numpy as np
import random
#import imutils
from pymavlink import mavutil
from datetime import datetime
#import smbus





time.sleep(1)
print("Connecting...")
#~ chemin_drone = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0'
chemin_drone = '/dev/ttyACM0'
#~ drone = connect(chemin_drone, wait_ready=True, baud=57600, heartbeat_timeout=2)

#chemin_drone = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0'
drone = connect(chemin_drone, wait_ready=False, baud=57600, heartbeat_timeout=2)
#drone = connect('127.0.0.1:14550', wait_ready=True)

#cap = cv2.VideoCapture(0)
def get_attitude():
	return drone.attitude
	
def get_location():
	return drone.location.global_relative_frame

while True:
  # Capture frame-by-frame
  #ret, frame = cap.read()
  
  altitudeDeVol = get_location().alt
  attitude = get_attitude()
  print ("drone.location.global_relative_frame.alt "+str(altitudeDeVol))
  print("drone.location.global_frame.alt "+str(drone.location.global_frame.alt))
  print("drone.location.local_frame.down "+str(drone.location.local_frame.down))
  print("drone.rangefinder.distance "+str(drone.rangefinder.distance))
  #print("nbr pixel pour 25 cm "+str(int(102/drone.rangefinder.distance)))
  #print("roll "+str(math.degrees(attitude.roll)))
  #print("pitch "+str(math.degrees(attitude.pitch)))
  print("/")
  
  # drone is an instance of the drone class
  print ("Autopilot Firmware version: %s" % drone.version)
  #print ("Autopilot capabilities (supports ftp): %s" % drone.capabilities.ftp)
  print ("Global Location: %s" % drone.location.global_frame)
  print ("Global Location (relative altitude): %s" % drone.location.global_relative_frame)
  print ("Local Location: %s" % drone.location.local_frame)    #NED
  print ("Attitude: %s" % drone.attitude)
  print ("Velocity: %s" % drone.velocity)
  print ("GPS: %s" % drone.gps_0)
  print ("Groundspeed: %s" % drone.groundspeed)
  print ("Airspeed: %s" % drone.airspeed)
  print ("Gimbal status: %s" % drone.gimbal)
  print ("Battery: %s" % drone.battery)
  print ("EKF OK?: %s" % drone.ekf_ok)
  print ("Last Heartbeat: %s" % drone.last_heartbeat)
  print ("Rangefinder: %s" % drone.rangefinder)
  print ("Rangefinder distance: %s" % drone.rangefinder.distance)
  print ("Rangefinder voltage: %s" % drone.rangefinder.voltage)
  print ("Heading: %s" % drone.heading)
  print ("Is Armable?: %s" % drone.is_armable)
  print ("System status: %s" % drone.system_status.state)
  print ("Mode: %s" % drone.mode.name)    # settable
  print ("Armed: %s" % drone.armed)    # settable
  
  print("/")
  print("/")
  print("Global Location (relative altitude) latitude : %s" % drone.location.global_relative_frame.lat)
  print("Global Location (relative altitude) longitude: %s" % drone.location.global_relative_frame.lon)
  print("/")
  print("/")
  
  #cv2.line(frame,(320,240),(320,240-int(60/drone.rangefinder.distance)),(0,255,0),1)
  
  #cv2.imshow('frame',frame)
  #if cv2.waitKey(1) & 0xFF == ord('q'):
  #	break
  
  
  
  
  
  time.sleep(2)
