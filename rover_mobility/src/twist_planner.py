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
from geometry_msgs.msg import Twist
import utm
import math
#---------------------------------------------------- 
#SIGINT Handler to escape loops. Use Ctrl-C to exit
def sigint_handler(signal, frame):
	pdb.set_trace()
	sys.exit(0)
#----------------------------------------------------

def get_speed():
	d = sqrt((utm_coor[0]-final_utm[0])**2 + (utm_coor[1]-final_utm[1])**2)
	d_thres = 5
	if(d<d_thres):
		return(0)	
	else:
		return(d)

def final_heading():
	t_slope = (utm_coor[1]-final_utm[1])/(utm_coor[0]-final_utm[0])
	theta = math.atan(t_slope)*180/np.pi
	return theta
#----------------------------------------------------

def twist_pub_callback(inp):
	global break_loop
	global twist_msg
	global in_linear_mode
	if(inp.data == 0):
		print("Stop Ack_received")
		rospy.loginfo("Rotated to desired heading.Start publishing linear speed")
		twist.angular.x = 0
		twist.linear.x = get_speed()
		in_linear_mode = True
		twist_pub.publish(twist_msg)
		print("First linear speed sent")
	# if(inp.data == 1):
	# 	print("Previous speed command received. Send next Command")
	# 	twist.angular.x = 0
	# 	twist.linear.x = get_speed()
	# 	twist_pub.publish(twist_msg)
	# 	rospy.loginfo("Another speed cpmmand sent")
	if(inp.data == 2):
		print("Autonomous Mode started")
		twist.angular.x = final_heading()
		twist.linear.x = 0
		twist_pub.publish(twist_msg)
	if(inp.data == 3):
		print("Reached Final Destination")
		in_linear_mode = False
		break_loop = True	
	return

def gps_callback(inp):
	global utm_coor
	utm_coor = utm.to_LatLon(inp.data[0],inp.data[1])[:2]
	return

if __name__ == "__main__":

	signal.signal(signal.SIGINT, sigint_handler)
	utm_coor = [0.0,0.0]		#Find better initialization
	final_gps = [0.0,0.0]
	final_utm = utm.to_LatLon(final_gps[0],final_gps[1])[:2]
#------------------------------------------------------------------------------------------	
	rospy.init_node("twist_node",anonymous=True)
	rospy.loginfo("Starting twist node")
	r_time = rospy.Rate(10)
	twist_msg = Twist()
	twist_pub = rospy.Publisher("twist_msg",Twist,queue_size=10)
	rospy.Subscriber("/twist_ack",Float32,twist_pub_callback)	#subcriber to receive twist acks
	rospy.Subscriber("/GPS",Float32MultiArray,gps_callback)		#subscriber to receive gps coordinates
#-------------------------------------------------------------------------------------------	
	in_linear_mode = False
	r_time.sleep()
	break_loop=False
	while not break_loop:
		if(in_linear_mode):
			twist.angular.x = 0
			twist.linear.x = get_speed()
			twist_pub.publish(twist_msg)
		r_time.sleep()