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
# import utm
#---------------------------------------------------- 


class Drive:

	def __init__(self,driver1,driver2):

		self.mode = "joystick"  #joystick or autonomous
		self.speed = 0   #Speed of wheels	
		self.direction = "forward"	#string that will hold either forward,backward,left or right
		self.frontClaw = driver1	#initialising with the roboclaw. One time use
		self.rearClaw = driver2
		self.rest = True
		self.drive_scale = 200   #Change according to tests
		self.turn_scale = 50
		self.stop()
		self.currents = [0,0,0,0]  #fl,fr,bl,br order of roboclaw currents
		self.current_threshold = 2000 #needs to be tuned while testing.#1500 means 1500*(10 mA) = 15A
		self.posx = 0.0
		self.posy = 0.0
		self.velx = 0.0
		self.vely = 0.0
		self.intialized = False
		self.curr_heading = 0
		self.final_heading = 0	#range is -180 to 180
		self.prev_err=0
		self.Iterm=0
		self.Iterm_windout=0
		self.heading_list=[]
		self.angle_threshold = 5
		self.initialized = False;
		self.ack_pub = rospy.Publisher('/twist_ack',Float32,queue_size=10)
		rate_ack = rospy.Rate(10)
		self.state = "angle mode"

	def fwd(self):
		self.frontClaw.ForwardM1(self.speed)
		#self.rearClaw.ForwardM1(self.speed)	#error on mech side
		self.rearClaw.BackwardM1(self.speed)
		self.frontClaw.ForwardM2(self.speed)
		self.rearClaw.ForwardM2(self.speed)

	def bwd(self):
		self.frontClaw.BackwardM1(self.speed)
		#self.rearClaw.BackwardM1(self.speed)
		self.rearClaw.ForwardM1(self.speed)
		self.frontClaw.BackwardM2(self.speed)
		self.rearClaw.BackwardM2(self.speed)

	def right(self):
		self.frontClaw.BackwardM1(self.speed)
		#self.rearClaw.BackwardM1(self.speed)
		self.rearClaw.ForwardM1(self.speed)
		self.frontClaw.ForwardM2(self.speed)
		self.rearClaw.ForwardM2(self.speed)

	def left(self):
		self.frontClaw.ForwardM1(self.speed)
		#self.rearClaw.ForwardM1(self.speed)
		self.rearClaw.BackwardM1(self.speed)
		self.frontClaw.BackwardM2(self.speed)
		self.rearClaw.BackwardM2(self.speed)

	def stop(self):
		self.frontClaw.ForwardM1(0)
		self.rearClaw.ForwardM1(0)
		self.frontClaw.ForwardM2(0)
		self.rearClaw.ForwardM2(0)						

	def update_steer(self):
		
		if(self.rest):
			self.stop()
		else:
			if(self.direction == "forward"):
				print (self.speed)
				print (self.direction)
				print(self.rest)
				self.fwd()
			elif (self.direction == "backward"):
				self.bwd()
			elif (self.direction == "left"):
				self.left()
			else:
				self.right()			
					

	def drive_callback(self,inp):
		#rospy.loginfo("ENTERED CALLBACK")
		axes = inp.axes				#axe[1] will control forward and backward speed. axe[3] will control turn direction and speed
		buttons = inp.buttons		#kept fot further use
		if(buttons[7] == 1):
			self.mode= "autonomous" if (self.mode == "joystick") else "joystick"
			print("mode changed")
		if(self.mode == "autonomous"):
			self.ack_pub.publish(2)
			return			
		if(axes[1]<0.1 and axes[1]>-(0.1) and axes[3]<0.1 and axes[3]>-0.1):
			self.speed = 0
			self.rest = True

		elif (axes[1]>0.1 or axes[1]<-0.1):
			self.rest = False
			self.speed = int(min(255,axes[1]*axes[1]*(self.drive_scale)))	
			if(axes[1] > 0):
				self.direction = "backward"
			else:	
				self.direction = "forward"

		elif (axes[3]>0.1 or axes[3]<-0.1):
			self.rest = False
			self.speed = int(min(255,axes[3] * axes[3] * (self.turn_scale)))
			if(axes[3] > 0):
				self.direction = "left"
			else:
				self.direction = "right"

	def imu_callback(self,inp):
		if(self.state != "angle mode"):
			#print("no imu use")
			return
		self.heading = inp.data[2]*180/np.pi
		self.heading_list.append(self.heading)
		#print("Current :"+str(self.heading) + " Final: "+str(self.final_heading))
		if(self.mode == "joystick"):
			self.final_heading=self.heading
			return		
		#self.angle_threshold = 3
		# kp = 
		# kd = 0
		# ki = 0
		elif(self.initialized == False):
			print("returning")
			return
		#print("gone ahead")
		angle_diff = self.final_heading - self.heading
		if(abs(angle_diff) < self.angle_threshold):
			self.speed = 0
			self.rest = True
			#print("in threshold")
			if((np.abs((np.array(self.heading_list[::-1][0:10])-self.final_heading))<self.angle_threshold).all()):
				#self.mode = "joystick"
				self.ack_pub.publish(0)
				self.state = "forward"	
				#print("converged to given angle")
		elif(abs(angle_diff)  < 180):
			print("rotating")
			self.converged = False
			self.rest = False
			self.speed = int(40*np.exp(-(abs(angle_diff)-20))+50 if abs(angle_diff)>20\
			 else 70*np.exp((abs(angle_diff)-20)/4)+20)
			if(angle_diff<0):
				self.direction = "right"
			else:
				self.direction = "left"
			print("rotating")
			self.prev_err=angle_diff					
		else:
			print("rotating")
			self.rest = False
			mod_angle_diff=360-abs(angle_diff)
			self.speed = int(40*np.exp(-(abs(mod_angle_diff)-20))+50 if abs(mod_angle_diff)>20\
			 else 70*np.exp((abs(mod_angle_diff)-20)/4)+20)
			if(angle_diff<0):
				self.direction = "left"
			else:
				self.direction = "right"
			print("rotating")	
			self.prev_err=mod_angle_diff			
		#print(self.mode)
		#print(self.speed)
		#print(self.direction)
		return

	def twist_callback(self , inp):
		twist_lin_k = 0.00005
		lin_speed = inp.linear.x #Check which of the three is the linear speed
		angle_rot = inp.angular.x #Check which of the three is angle of roattion 
		if (angle_rot > self.angle_threshold):
			self.final_heading = angle_rot #+ self.heading #Uncomment this if publishinf angle of rotation
			#self.rest = False
			self.initialized = True;
			print("initialized");
		else:
			if(lin_speed > 0):
				self.speed = int(min(255,(twist_lin_k*lin_speed)))
				self.direction = 'forward'
				self.rest = False
				print("received speed:")
				print (self.speed)
				print (self.direction)
				print(self.rest)
			else:	
				self.rest = True
				self.speed = 0
				self.ack_pub.publish(3)
				print ("reached_final_destination")	


	def current_limiter(self):
		(i,self.currents[0],self.currents[1]) = self.frontClaw.ReadCurrents()
		(i,self.currents[2],self.currents[3]) = self.rearClaw.ReadCurrents()
		for i in range(4):
			if(int(self.currents[i]) > self.current_threshold):
				self.stop()
				return True
		return False		

#---------------------------------------------------


