#!/usr/bin/env python
from Rover_Drive_class import *
from roboclaw import RoboClaw
import rospy
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
	Rover.stop()
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
			frontDrive = RoboClaw(0x80, "/dev/frontDrive", 9600)
			break;
		except SerialException:
			rospy.logwarn("Couldn't connect to Front Drive Claw. trying again")
			r_time.sleep()
	rospy.loginfo("Connected to Front Drive Claw")
	while(True):
		try:
			backDrive = RoboClaw(0x80, "/dev/backDrive", 9600)
			break;
		except SerialException:
			rospy.logwarn("Couldn't connect to Back Drive Claw. trying again")
			r_time.sleep()
	rospy.loginfo("Connected to Back Drive Claw")
	while(True):
		try:
			frontSteer = RoboClaw(0x80, "/dev/frontSteer", 9600)
			break;
		except SerialException:
			rospy.logwarn("Couldn't connect to Front Steer Claw. trying again")
			r_time.sleep()
	rospy.loginfo("Connected to Front Steer Claw")
	while(True):
		try:
			backSteer = RoboClaw(0x80, "/dev/backSteer", 9600)
			break;
		except SerialException:
			rospy.logwarn("Couldn't connect to Back Steer Claw. trying again")
			r_time.sleep()
	rospy.loginfo("Connected to Back Steer Claw")
	#connected---------------------------------------

	#initialising Drive object-------------------
	Rover = Rover_drive(frontDrive,backDrive,frontSteer,backSteer);
	#subscriber lines--------------------------------------------------	
	#ros::Subscriber joy_sub = _nh.subscribe("/joy", 100, joyCallback);
	rospy.Subscriber("/joy",Joy,Rover.indiv_callback)

	#updating the received intructions
	r_time_f=rospy.Rate(10)
	stopped = False
	while not rospy.is_shutdown():
		if(stopped == False):
			Rover.update_indiv_steer()
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