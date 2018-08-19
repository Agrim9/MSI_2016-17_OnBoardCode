from roboclaw import RoboClaw
import rospy
import tf
from std_msgs.msg import Float64MultiArray
import numpy as np
import signal
import sys
from serial.serialutil import SerialException as SerialException

#---------------------------------------------------- 
#SIGINT Handler to escape loops. Use Ctrl-C to exit
def sigint_handler(signal, frame):
    sys.exit(0)
#----------------------------------------------------    

class New_Drive:

	def __init__(self,driver1,driver2):
		
		self.speed = 0   #Speed of wheels	
		self.direction = "forward"	#string that will hold either forward,backward,left or right
		self.frontClaw = driver1	#initialising with the roboclaw. One time use
		self.rearClaw = driver2
		self.rest = True
		self.drive_scale = 300   #Change according to tests
		self.turn_scale = 300
		self.stop()
		self.currents = [0,0,0,0]  #fl,fr,bl,br order of roboclaw currents

	def fwd(self):
		self.frontClaw.ForwardM1(self.speed)
		self.rearClaw.ForwardM1(self.speed)
		self.frontClaw.ForwardM2(self.speed)
		self.rearClaw.ForwardM2(self.speed)

	def bwd(self):
		self.frontClaw.BackwardM1(self.speed)
		self.rearClaw.BackwardM1(self.speed)
		self.frontClaw.BackwardM2(self.speed)
		self.rearClaw.BackwardM2(self.speed)

	def left(self):
		self.frontClaw.BackwardM1(self.speed)
		self.rearClaw.BackwardM1(self.speed)
		self.frontClaw.ForwardM2(self.speed)
		self.rearClaw.ForwardM2(self.speed)

	def right(self):
		self.frontClaw.ForwardM1(self.speed)
		self.rearClaw.ForwardM1(self.speed)
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
				self.fwd()
			elif (self.direction == "backward"):
				self.bwd()
			elif (self.direction == "left"):
				self.left()
			else:
				self.right()			
					

	def drive_callback(self,inp):
		
		axes = inp.axes				#axe[1] will control forward and backward speed. axe[3] will control turn direction and speed
		buttons = inp.buttons		#kept fot further use
		if(axes[1]<threshold and axes[1]>-(threshold) and axes[3]<threshold and axes[3]>-threshold):
			self.speed = 0
			self.rest = True

		elif (axes[1]>threshold or axes[1]<-threshold):
			self.rest = False
			self.speed = min(255,axes[1]*axes[1]*(self.drive_scale))	
			if(axes[1] > 0):
				self.direction = "forward"
			else:
				self.direction = "backward"

		elif (axes[3]>threshold or axes[3]<-threshold):
			self.rest = False
			self.speed = min(255,axes[3] * axes[3] * (self.turn_scale))
			if(axes[3] > 0):
				self.direction = "left"
			else
				self.direction = "right"

	def current_limiter(self):
		(i,self.currents[0],self.currents[1]) = self.frontClaw.ReadCurrents()
		(i,self.currents[2],self.currents[3]) = self.rearClaw.ReadCurrents()
		for i in range(4):
			if(self.currents[i] > current_threshold):
				self.stop()
				return True


#---------------------------------------------------


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
			frontClaw = RoboClaw(0x80, "/dev/frontClaw", 9600)
			break;
		except SerialException:
			rospy.logwarn("Couldn't connect. trying again")
			r_time.sleep()
	rospy.loginfo("Connected to RoboClaw1")
	while(True):
		try:
			rearClaw = RoboClaw(0x80, "/dev/rearClaw", 9600)
			break;
		except SerialException:
			rospy.logwarn("Couldn't connect. trying again")
			r_time.sleep()
	rospy.loginfo("Connected to RoboClaw2")
	#connected---------------------------------------

	#initialising New_Drive object-------------------
	new_drive = New_Drive(frontClaw,rearClaw)
	#added self.stop in __init__. Add seperately her if it doesnt work
	#------------------------------------------------

	#subscriber lines--------------------------------------------------
	#ros::Subscriber joy_sub = _nh.subscribe("/joy", 100, joyCallback);
	rospy.Subscriber("/joy",Float64MultiArray,drive_callback)
	#-------------------------------------------------------------------
	
	#-------------------------------------------------------------------
	#updating the received intructions
	r_time_f=rospy.Rate(10)
	stopped = False
    while not rospy.is_shutdown():
    	if(stopped = False):
        	diffClaw.update_steer()
        else
        	print("stopped due to excess current")	
        if(diffClaw.current_limiter()):
        	stopped = True
        	rospy.loginfo(new_drive.currents)
        rospy.loginfo(new_drive.direction)
        rospy.loginfo(new_drive.speed)
        r_time_f.sleep()
    #-------------------------------------------------------------------    