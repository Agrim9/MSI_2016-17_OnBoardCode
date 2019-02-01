#!/usr/bin/env python
from roboclaw import RoboClaw
import rospy
import tf
from std_msgs.msg import Float64MultiArray,Float32MultiArray,Float32
from sensor_msgs.msg import Joy
import numpy as np
import signal
import sys
from serial.serialutil import SerialException as SerialException
import pdb
import utm
import math
from full_drive import *
#---------------------------------------------------- 
#SIGINT Handler to escape loops. Use Ctrl-C to exit
def sigint_handler(signal, frame):
	pdb.set_trace()
	sys.exit(0)
#----------------------------------------------------

def get_angspeed():
	angle_diff = (final_heading() - curr_head)
	if(np.abs(angle_diff)<3):
		return 0
	else:
		return 200

def get_speed():
	d = math.sqrt((utm_coor[0]-final_utm[0])**2 + (utm_coor[1]-final_utm[1])**2)
	const_p = 10
	d = d*const_p
	d_thres = 5
	if(d<=d_thres):
		return 0	
	else:
		return d

def final_heading():
	t_slope = (utm_coor[1]-final_utm[1])/(utm_coor[0]-final_utm[0])
	theta = math.atan(t_slope)*180/np.pi
	return 0
#----------------------------------------------------

def twist_pub_callback(inp):
	global break_loop
	global twist_msg
	global in_linear_mode
	if(gps_init == False):
		return
	if(inp.data == 0):
		print("Stop Ack_received")
		rospy.loginfo("Rotated to desired heading.Start publishing linear speed")
		twist_msg.angular.x = 0
		twist_msg.linear.x = get_speed()
		in_linear_mode = True
		twist_pub.publish(twist_msg)
		print("First linear speed sent")
	if(inp.data == 2):
		print("Autonomous Mode started")
		twist_msg.angular.x = final_heading()
		twist_msg.linear.x = 0
		twist_pub.publish(twist_msg)
		print("Angle sent")
	if(inp.data == 3):
		print("Reached Final Destination")
		in_linear_mode = False
		break_loop = True	

def gps_callback(inp):
	global utm_coor, GPS_obtained
	utm_coor = utm.from_latlon(inp.data[0],inp.data[1])[:2]
	if(not GPS_obtained):
		GPS_obtained=True

def imu_callback(inp):
	global curr_head,IMU_obtained
	curr_head = inp.data[2]*180/np.pi	
	if(not IMU_obtained):
		IMU_obtained=True

if __name__ == "__main__":

	signal.signal(signal.SIGINT, sigint_handler)
	utm_coor = [0.0,0.0]		#Find better initialization
	final_gps = [19.13118345, 72.91830041]
	final_utm = utm.from_latlon(final_gps[0],final_gps[1])[:2]
	
	#------------------------------------------------------------------------------------------	
	curr_head = 0
	gps_init = False
	rospy.init_node("Lvl 3 node",anonymous=True)
	r_time = rospy.Rate(10)
	rospy.Subscriber("LatLon",Float64MultiArray,gps_callback)		#subscriber to receive gps coordinates
	rospy.Subscriber("IMU",Float32MultiArray,imu_callback)
	IMU_obtained=False
	GPS_obtained=False
	#-------------------------------------------------------------------------------------------	
	mode = "turn"
	r_time.sleep()
	break_loop=False
	#------------------------------------------------
	#Trying to connect to roboclaw drivers 1 and 2
	while(True):
		try:
			rightClaw = RoboClaw(0x80, "/dev/rightClaw", 9600)
			break;
		except SerialException:
			rospy.logwarn("Couldn't connect to Right Claw. trying again")
			r_time.sleep()
	rospy.loginfo("Connected to Right Claw")
	while(True):
		try:
			leftClaw = RoboClaw(0x80, "/dev/leftClaw", 9600)
			break;
		except SerialException:
			rospy.logwarn("Couldn't connect to Left Claw. trying again")
			r_time.sleep()
	rospy.loginfo("Connected to Left Claw")
	#connected---------------------------------------

	#initialising Drive object-------------------
	Drive = Drive(rightClaw,leftClaw)
	#added self.stop in __init__. Add seperately her if it doesnt work
	#------------------------------------------------
	# stopped = False
	steer_time=1.5
	fix_obtained=False
	while not break_loop:
		if((not IMU_obtained) or (not GPS_obtained)):
			print(IMU_obtained)
			print("No GPS/IMU Fix")
			r_time.sleep()
			continue
		else:
			if(not fix_obtained):
				print("Final Heading should be"+str(final_heading()))
				fix_obtained=True

		if(mode=="linear"):
			curr_speed=get_speed()
			if(curr_speed==0):
				break_loop=True
				mode="Stop"
				continue
			Drive.autonom_ahead(curr_speed)
		elif(mode=="turn"):
			if((curr_head-final_heading())>0):
				turn_var=True
			else:
				turn_var=False
			Drive.autonom_turn(steer_time,turn_var)
			mode="IMU_check"
		elif(mode=="IMU_check"):
			ang_speed=get_angspeed()
			if(ang_speed==0):
				Drive.autonom_turn(steer_time,not turn_var)
				mode="linear"
				continue
			Drive.autonom_ahead(ang_speed)



r_time.sleep()