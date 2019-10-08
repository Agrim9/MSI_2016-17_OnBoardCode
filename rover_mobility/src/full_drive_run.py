#!/usr/bin/env python
from autonomous import *
from roboclaw import RoboClaw
import rospy
import tf
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
import numpy as np
import signal
import sys
from serial.serialutil import SerialException as SerialException
import pdb
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
	rospy.init_node("Drive_Node")
	rospy.loginfo("Starting drive node")
	r_time = rospy.Rate(1)

	#------------------------------------------------
	#Trying to connect to roboclaw drivers 1 and 2
	while(True):
		try:
			rightClaw = RoboClaw(0x80, "/dev/ttyACM1", 9600)
			break;
		except SerialException:
			rospy.logwarn("Couldn't connect to Right Claw at ttyACM1. trying again")
			r_time.sleep()
	rospy.loginfo("Connected to Right Claw")
	while(True):
		try:
			leftClaw = RoboClaw(0x80, "/dev/ttyACM0", 9600)
			break;
		except SerialException:
			rospy.logwarn("Couldn't connect to Left Claw at ttyACM0. trying again")
			r_time.sleep()
	rospy.loginfo("Connected to Left Claw")
	#connected---------------------------------------

	#initialising Drive object-------------------
	Drive = Drive(rightClaw,leftClaw)
	#added self.stop in __init__. Add seperately here if it doesnt work
	#------------------------------------------------

	#subscriber lines--------------------------------------------------
	#ros::Subscriber joy_sub = _nh.subscribe("/joy", 100, joyCallback);
	rospy.Subscriber("/joy",Joy,Drive.drive_callback)
	# rospy.Subscriber("/gps", Float32MultiArray, Drive.GPS)
	# rospy.Subscriber("/imu",Float32MultiArray, Drive.IMU)
	#-------------------------------------------------------------------
	
	#-------------------------------------------------------------------
	#updating the received intructions
	r_time_f=rospy.Rate(10)
	stopped = False
	while not rospy.is_shutdown():
		if(stopped == False):
			Drive.update_steer()
			# Drive.update_turn()
		else:
			print("stopped due to excess current")	
		#if(Drive.current_limiter()):			#uncomment after setting current_threshold appropriately
			print("CURRENT ERROR")
			stopped = True
			rospy.loginfo(Drive.currents)
		#rospy.loginfo(Drive.direction)
		#rospy.loginfo(Drive.speed)
		r_time_f.sleep()
	#-------------------------------------------------------------------    
	#left axes forward forward (as on 25th)
	#right axes forward left (as on 25th)