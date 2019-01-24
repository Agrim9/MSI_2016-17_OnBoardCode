#!/usr/bin/env python
#Contains codes for all drive related things. Imported in other codes for usage. 
from roboclaw import RoboClaw
import rospy
import tf
from std_msgs.msg import Float64MultiArray,Float32MultiArray,Float32
from sensor_msgs.msg import Joy
import numpy as np
import signal
import sys
from serial.serialutil import SerialException as SerialException
from geometry_msgs.msg import Twist
import utm
import RPi.GPIO as g
#---------------------------------------------------- 

class Drive:
	def __init__(self,driver1,driver2):
		self.rightClaw = driver1
		self.leftClaw = driver2
		self.rest = True
		self.direction = "forward" #forward or backward
		self.speed = 0
		self.turn_dir = "left"#left or right
		self.drivemode = "open_loop"
		self.turn = False

		g.setmode(g.BCM)
		g.setup(17,g.OUT)
		g.setup(22,g.OUT)
		g.setup(23,g.OUT)
		g.setup(24,g.OUT)
		g.setup(12,g.OUT)
		g.setup(13,g.OUT)
		g.setup(18,g.OUT)
		g.setup(19,g.OUT)
		self.pwm1 = g.PWM(18, 100)
		self.pwm2 = g.PWM(19, 100)
		self.pwm3 = g.PWM(13, 100)
		self.pwm4 = g.PWM(12, 100)
		self.pwm1.start(0)
		self.pwm2.start(0)
		self.pwm3.start(0)
		self.pwm4.start(0)

	def fwd(self):
		self.rightClaw.ForwardM1(self.speed)
		self.rightClaw.BackwardM2(self.speed)
		self.leftClaw.BackwardM1(self.speed)
		self.leftClaw.ForwardM2(self.speed)

	def bwd(self):
		self.rightClaw.BackwardM1(self.speed)
		self.rightClaw.ForwardM2(self.speed)
		self.leftClaw.ForwardM1(self.speed)
		self.leftClaw.BackwardM2(self.speed)

	def stop(self):
		self.rightClaw.ForwardM1(0)
		self.leftClaw.ForwardM1(0)
		self.rightClaw.ForwardM2(0)
		self.leftClaw.ForwardM2(0)

	def right(self):
		g.output(17,True)
		g.output(22,True)
		g.output(23,True)
		g.output(24,True)
		self.pwm1.ChangeDutyCycle(50)
		#self.pwm2.ChangeDutyCycle(50)
		#self.pwm3.ChangeDutyCycle(50)
		self.pwm4.ChangeDutyCycle(50)

	def left(self):
		g.output(17,False)
		g.output(22,False)
		g.output(23,False)
		g.output(24,False)
		self.pwm1.ChangeDutyCycle(50)
		#self.pwm2.ChangeDutyCycle(50)
		#self.pwm3.ChangeDutyCycle(50)
		self.pwm4.ChangeDutyCycle(50)		

	def turn_stop(self):
		g.output(18, False)
		g.output(19, False)
		g.output(13, False)
		g.output(12, False)
		self.pwm1.ChangeDutyCycle(0)
		self.pwm2.ChangeDutyCycle(0)
		self.pwm3.ChangeDutyCycle(0)
		self.pwm4.ChangeDutyCycle(0)


	def update_steer(self):
		if(self.rest == True):
			self.stop()
		elif(self.direction == "forward"):
			self.fwd()
		else:
			self.bwd()

	def update_turn(self):
		if(self.turn == False):
			self.turn_stop()
		elif(self.turn_dir == "left"):
			self.left()
		else:
			self.right()				

	def drive_callback(self,inp):
		axes = inp.axes
		buttons = inp.buttons
		if(self.drivemode == "open_loop"):
			if(axes[1]<0.1 and axes[1]>-(0.1) and axes[3]<0.1 and axes[3]>-0.1):
				self.speed = 0
				self.rest = True
				self.turn = False
			elif (axes[1]>0.1 or axes[1]<-0.1):
				if(self.turn == True):
					self.rest = True
					return
				self.rest = False
				self.speed = int(min(255,400*axes[1]))	
				if(axes[1] > 0):
					self.direction = "forward"
				else:	
					self.direction = "backward"
			else:
				self.turn = True
				if(axes[3] > 0):
					self.turn_dir = "left"
				else:
					self.turn_dir = "right"	

	def current_limiter(self):
		(i,self.currents[0],self.currents[1]) = self.frontClaw.ReadCurrents()
		(i,self.currents[2],self.currents[3]) = self.rearClaw.ReadCurrents()
		for i in range(4):
			if(int(self.currents[i]) > self.current_threshold):
				self.stop()
				return True
		return False		

#---------------------------------------------------				