#!/usr/bin/env python
#Contains codes for all drive related things. Imported in other codes for usage. 
from roboclaw import RoboClaw
import rospy
from std_msgs.msg import Float64MultiArray,Float32MultiArray,Float32
from sensor_msgs.msg import Joy
import numpy as np
import signal
import sys
from serial.serialutil import SerialException as SerialException
from geometry_msgs.msg import Twist
import utm
import time
#----------------------------------------------------

class Rover_drive:

	def _init_(self,drivef,driveb,steerf,steerb):

		self.frontDrive = drivef
		self.backDrive = driveb
		self.frontSteer = steerf
		self.backSteer = steerb

		self.rest = True
		self.direction = "forward"
		self.drivespeed = 0
		self.steerspeed = 0

		def fwd(self):
			self.frontDrive.ForwardM1(self.drivespeed)
			self.frontDrive.ForwardM2(self.drivespeed)
			self.backDrive.ForwardM1(self.drivespeed)
			self.backDrive.ForwardM2(self.drivespeed)

		def bwd(self):
			self.frontDrive.BackwardM1(self.drivespeed)
			self.frontDrive.BackwardM2(self.drivespeed)
			self.backDrive.BackwardM1(self.drivespeed)
			self.backDrive.BackwardM2(self.drivespeed)

		def turn_right(self):
			self.frontSteer.ForwardM1(self.turnspeed)
			self.frontSteer.ForwardM2(self.turnspeed)

		def turn_left(self):
			self.frontSteer.BackwardM1(self.turnspeed)
			self.frontSteer.BackwardM2(self.turnspeed)

		def restore(self): #ReadEncM1
			kp = 2
			ki = 0.01
			kd = 0.01
			preverr1 = 0
			preverr2 = 0
			err_threshold = 2 #Change this threshold 
			while(True):
				err1 = ReadEncM1(self)
				err2 = ReadEncM2(self)

