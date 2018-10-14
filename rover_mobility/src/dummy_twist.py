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

#---------------------------------------------------- 
#SIGINT Handler to escape loops. Use Ctrl-C to exit
def sigint_handler(signal, frame):
	pdb.set_trace()
	sys.exit(0)
#----------------------------------------------------

def twist_pub_callback(inp):
	global break_loop
	if(inp.data == 0):
		print("Stop Ack_received")
		rospy.loginfo("Stop Program")
		break_loop=True
	if(inp.data == 1):
		print("Command Ack_received")
		rospy.loginfo("Twist_ack Received")
	if(inp.data == 2):
		print("Autonom Mode started")
		twist_pub.publish(twist_msg)
	return	

if __name__ == "__main__":

	signal.signal(signal.SIGINT, sigint_handler)
	rospy.init_node("twist_node",anonymous=True)
	rospy.loginfo("Starting twist node")
	r_time = rospy.Rate(10)
	twist_msg = Twist()
	twist_msg.angular.x = 30 #Alawys roate it by 30 degrees
	twist_pub = rospy.Publisher("twist_msg",Twist,queue_size=10)
	rospy.Subscriber("/twist_ack",Float32,twist_pub_callback)
	r_time.sleep()
	break_loop=False
	while not break_loop:
		r_time.sleep()