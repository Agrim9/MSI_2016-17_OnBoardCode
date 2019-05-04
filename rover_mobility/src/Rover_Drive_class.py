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

		def __init__(self,drivef,driveb,steerf,steerb):
			self.frontDrive = drivef
			self.backDrive = driveb
			self.frontSteer = steerf
			self.backSteer = steerb
			self.indiv_state="fr"
			self.rest = True
			self.turn = False
			self.direction = "forward"
			self.drivespeed = 0
			self.steerspeed = 0
			self.restore_done = True

			self.init1 = self.frontSteer.ReadEncM1()[1]
			self.init2 = self.frontSteer.ReadEncM2()[1]

		def stop(self):
			self.frontDrive.ForwardM1(0)
			self.frontDrive.ForwardM2(0)
			self.backDrive.ForwardM1(0)
			self.backDrive.ForwardM2(0)
			self.frontSteer.ForwardM1(0)
			self.frontSteer.ForwardM2(0)
			self.backSteer.ForwardM1(0)
			self.backSteer.ForwardM2(0)	

		def bwd(self):
			self.frontDrive.ForwardM1(self.drivespeed)
			self.frontDrive.ForwardM2(self.drivespeed)
			self.backDrive.ForwardM1(self.drivespeed)
			self.backDrive.ForwardM2(self.drivespeed)

		def fwd(self):
			self.frontDrive.BackwardM1(self.drivespeed)
			self.frontDrive.BackwardM2(self.drivespeed)
			self.backDrive.BackwardM1(self.drivespeed)
			self.backDrive.BackwardM2(self.drivespeed)

		def turn_left(self,indiv=False):
			if(not indiv):
				self.frontSteer.ForwardM1(self.turnspeed)
				self.frontSteer.BackwardM2(self.turnspeed)
			else:
				if(self.indiv_state=="fr"):
					self.frontSteer.BackwardM2(self.turnspeed)
				elif(self.indiv_state=="fl"):
					self.frontSteer.ForwardM1(self.turnspeed)
				elif(self.indiv_state=="br"):
					self.backSteer.BackwardM2(self.turnspeed)
				elif(self.indiv_state=="bl"):
					self.backSteer.ForwardM1(self.turnspeed)

		def turn_right(self,indiv=False):
			if(not indiv):
				self.frontSteer.BackwardM1(self.turnspeed)
				self.frontSteer.ForwardM2(self.turnspeed)
			else:
				if(self.indiv_state=="fr"):
					self.frontSteer.ForwardM2(self.turnspeed)
				elif(self.indiv_state=="fl"):
					self.frontSteer.BackwardM1(self.turnspeed)
				elif(self.indiv_state=="br"):
					self.backSteer.ForwardM2(self.turnspeed)
				elif(self.indiv_state=="bl"):
					self.backSteer.BackwardM1(self.turnspeed)

		def restore(self): #ReadEncM1
			kp = 5
			ki = 1
			kd = 0.1
			preverr1 = 0
			preverr2 = 0
			integ1 = 0
			integ2 = 0
			err_threshold = 100 #Change this threshold 
			while(True):
				err1 = self.frontSteer.ReadEncM1()[1] - self.init1
				err2 = self.frontSteer.ReadEncM2()[1] - self.init2
				if(abs(err1)<err_threshold and abs(err2)<err_threshold):
					break
				else:
					if(abs(err1) < err_threshold):
						err1 = 0
						integ1 = 0
						preverr1 = 0
					if(abs(err2) < err_threshold):
						err2 = 0
						integ2 = 0
						preverr2 = 0
					integ1 = integ1 + err1
					integ2 = integ2 + err2
					d1 = err1 - preverr1
					d2 = err2 - preverr2
					preverr1 = err1
					preverr2 = err2	
					speed1 = int(min(155,kp*err1+ki*integ1+kd*d1))
					speed2 = int(min(155,kp*err2+ki*integ2+kd*d2))
					if(err1>0):
						self.frontSteer.BackwardM1(speed1)
					elif(err1<0):
						self.frontSteer.ForwardM1(speed1)
					if(err2>0):
						self.frontSteer.ForwardM2(speed2)
					elif(err2<0):
						self.frontSteer.BackwardM2(speed2)
			self.restore_done = True
			self.direction = "forward"
			self.drivespeed = 0
			self.turnspeed = 0
			print("restored")			

		def update_drive(self):
			if(self.direction == "restore"):
				self.restore()
			elif(self.rest == False):
				if(self.turn == True):
					if(self.direction == "left"):
						self.turn_left()
					elif(self.direction == "right"):
						self.turn_right()
				else:
					if(self.direction == "forward"):
						self.fwd()
					elif(self.direction == "backward"):
						self.bwd()
			else:
				self.stop()				

		def update_indiv_steer(self):
			if(self.rest == False):
				if(self.turn == True):
					if(self.direction == "left"):
						self.turn_left(indiv=True)
					elif(self.direction == "right"):
						self.turn_right(indiv=True)
			else:
				self.stop()				

		def drive_callback(self,inp):
			axes = inp.axes
			buttons = inp.buttons
			if(self.restore_done == False):
				return
			elif(buttons[8] == 1):
				self.direction = "restore"
				print("restoring")
			elif(axes[1]<0.1 and axes[1] > -0.1 and axes[2]<0.1 and axes[2] > -0.1):
				self.drivespeed = 0
				self.turnspeed = 0
				self.rest = True
				self.turn = False	
			elif (axes[1]>0.1 or axes[1]<-0.1):
				if(self.turn == False):
					self.rest = False
					self.turnspeed = 0
					self.drivespeed = int(min(255,400*axes[1]))
					if(axes[1]>0):
						self.direction = "forward"
					else:
						self.direction = "backward"
			elif (axes[2]>0.1 or axes[2]<-0.1):
				self.turn = True
				self.rest = False
				self.drivespeed = 0
				self.turnspeed = int(min(255,30*axes[2]*axes[2]))
				if(axes[2]>0):
					self.direction = "left"
				else:
					self.direction = "right"
			else:
				self.drivespeed = 0
				self.turnspeed = 0
				self.rest = True
				self.turn = False


		def indiv_callback(self,inp):
			axes = inp.axes
			buttons = inp.buttons
			if(axes[4]==1):
				print("Shifting to FL")
				self.indiv_state="fl"
			elif(axes[4]==-1):
				print("Shifting to FR")
				self.indiv_state="fr"
			elif(buttons[0]==1):
				print("Shifting to BL")
				self.indiv_state="bl"
			elif(buttons[2]==1):
				print("Shifting to BR")
				self.indiv_state="br"
			elif (axes[2]>0.1 or axes[2]<-0.1):
				self.rest = False
				self.turn = True
				self.turnspeed = int(min(255,30*axes[2]*axes[2]))
				if(axes[2]>0):
					self.direction = "left"
				else:
					self.direction = "right"


		def current_limiter(self):
			(i,self.currents[0],self.currents[1]) = self.frontClaw.ReadCurrents()
			(i,self.currents[2],self.currents[3]) = self.rearClaw.ReadCurrents()
			for i in range(4):
				if(int(self.currents[i]) > self.current_threshold):
					self.stop()
					return True
			return False        

#---------------------------------------------------                			