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
		self.turn = False
		self.direction = "forward"
		self.drivespeed = 0
		self.steerspeed = 0
		self.restore_done = True

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
			integ1 = 0
			integ2 = 0
			err_threshold = 2 #Change this threshold 
			while(True):
				err1 = ReadEncM1(self)
				err2 = ReadEncM2(self)
				if(abs(err1)<100 && abs(err2)<100):
					break
				else:
					if(err1 < 100) err1 = 0
					if(err2 < 100) err2 = 0
					speed1 = int(min(255,kp*abs(err1*255/4000)))
					speed2 = int(min(255,kp*abs(err2*255/4000)))
					if(err1>0):
						self.frontSteer.BackwardM1(speed1)
					else:
						self.frontSteer.ForwardM1(speed1)
					if(err2>0):
						self.frontSteer.BackwardM2(speed2)
					else:
						self.frontSteer.ForwardM2(speed2)
			self.restore_done = True			

		def update_drive(self):
			if(self.direction == "restore"):
				restore()
			elif(self.rest == False):
				if(self.turn == True):
					if(self.direction == "left"):
						turn_left()
					elif(self.direction == "right"):
						turn_right()
				else:
					if(self.direction == "forward"):
						fwd()
					elif(self.direction == "backward"):
						bwd()	


		def drive_callback(self,inp):
			axes = inp.axes
			buttons = inp.buttons
			if(restore_done == False):
				return
			elif(buttons[7] == 1)
				self.direction = "restore"
			elif (axes[1]>0.1 || axes[1]<-0.1):
				if(self.turn == False):
					self.rest = False
					self.turnspeed = 0
					self.drivespeed = int(min(255,400*axes[1]))
					if(axes[1]>0):
						self.direction = "forward"
					else:
						self.direction = "backward"
			elif (axes[3]>0.1 || axes[3]<-0.1):
				self.turn = True
				self.rest = False
				self.drivespeed = 0
				self.turnspeed = int(min(255,400*axes[3]))
				if(axes[3]>0):
					self.direction = "left"
				else:
					self.direction = "right"
			else:
				self.drivespeed = 0
                self.turnspeed = 0
                self.rest = True
				self.turn = False			



		def current_limiter(self):
        	(i,self.currents[0],self.currents[1]) = self.frontClaw.ReadCurrents()
        	(i,self.currents[2],self.currents[3]) = self.rearClaw.ReadCurrents()
        	for i in range(4):
            	if(int(self.currents[i]) > self.current_threshold):
              		self.stop()
                	return True
    	    return False        

#---------------------------------------------------                			