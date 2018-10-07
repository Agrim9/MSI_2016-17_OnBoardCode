#!/usr/bin/env python
from new_drive_codes import *
from roboclaw import RoboClaw
import rospy
import tf
from std_msgs.msg import Float64MultiArray,Float32MultiArray
from sensor_msgs.msg import Joy
import numpy as np
import signal
import sys
from serial.serialutil import SerialException as SerialException

#---------------------------------------------------- 
#SIGINT Handler to escape loops. Use Ctrl-C to exit
def sigint_handler(signal, frame):
	sys.exit(0)
#----------------------------------------------------


#Used for running the rover using joystick
#----------------------------------------------------
#main program
if __name__ == "__main__":

	signal.signal(signal.SIGINT, sigint_handler)
	rospy.init_node("Drive node")
	rospy.loginfo("Starting drive node")
	r_time = rospy.Rate(1)

	#------------------------------------------------
	#Trying to connect to roboclaw drivers 1 and 2
	while(True):
		try:
			frontClaw = RoboClaw(0x80, "/dev/frontClaw", 9600)
			break;
		except SerialException:
			rospy.logwarn("Couldn't connect to RoboClaw1. trying again")
			r_time.sleep()
	rospy.loginfo("Connected to RoboClaw1")
	while(True):
		try:
			rearClaw = RoboClaw(0x80, "/dev/rearClaw", 9600)
			break;
		except SerialException:
			rospy.logwarn("Couldn't connect to RoboClaw2. trying again")
			r_time.sleep()
	rospy.loginfo("Connected to RoboClaw2")
	#connected---------------------------------------

	#initialising Drive object-------------------
	Drive = Drive(frontClaw,rearClaw)
	#added self.stop in __init__. Add seperately her if it doesnt work
	#------------------------------------------------

	#subscriber lines--------------------------------------------------
	#ros::Subscriber joy_sub = _nh.subscribe("/joy", 100, joyCallback);
	rospy.Subscriber("/joy",Joy,Drive.drive_callback)
	rospy.Subscriber("/IMU",Float32MultiArray,Drive.imu_callback)
	#-------------------------------------------------------------------
	
	#-------------------------------------------------------------------
	#updating the received intructions
	r_time_f=rospy.Rate(10)
	stopped = False
	while not rospy.is_shutdown():
		if(stopped == False):
			Drive.update_steer()
		else:
			print("stopped due to excess current")	
		if(Drive.current_limiter()):			#uncomment after setting current_threshold appropriately
			print("CURRENT ERROR")
			stopped = True
			rospy.loginfo(Drive.currents)
		if(Drive.speed > 0):	
			rospy.loginfo(Drive.direction)
			rospy.loginfo(Drive.speed)
		r_time_f.sleep()
	#-------------------------------------------------------------------    
	#left axes forward forward (as on 25th)
	#right axes forward left (as on 25th)