from new_drive_codes import New_Drive
from roboclaw import RoboClaw
import rospy
import tf
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
import numpy as np
import signal
import sys
from serial.serialutil import SerialException as SerialException
import utm

#---------------------------------------------------- 
#SIGINT Handler to escape loops. Use Ctrl-C to exit
def sigint_handler(signal, frame):
	sys.exit(0)
#----------------------------------------------------

global end_posx,end_posy  
end_posx,end_posy = 10.0,10.0  #change end point here
#Used to run rover based on gps and imu
#Use to_latlon and from_latlon functions to convert to and from gps.

def distance_cartesian(x1, y1, x2, y2):
    dx = x1 - x2
    dy = y1 - y2

    return sqrt(dx * dx + dy * dy)

def distance_to_goal(posx,posy):
	return distance_cartesian(posx,posy,end_posx,end_posy)

#---------------------------------main program -----------------------------------------------
if __name__ == "__main__":\
	
	signal.signal(signal.SIGINT, sigint_handler)
	rospy.init_node("Autonomous node")
	rospy.loginfo("Starting autonomous node")
	r_time = rospy.Rate(1)

	#------------------------------------------------
	#Trying to connect to roboclaw drivers 1 and 2
	while(True):
		try:
			#frontClaw = RoboClaw(0x80, "/dev/frontClaw", 9600)
			frontClaw = RoboClaw(0x80, "/dev/ttyACM0", 9600)   #Port locking still to be done
			break;
		except SerialException:
			rospy.logwarn("Couldn't connect to RoboClaw1. trying again")
			r_time.sleep()
	rospy.loginfo("Connected to RoboClaw1")
	while(True):
		try:
			#rearClaw = RoboClaw(0x80, "/dev/rearClaw", 9600)
			rearClaw = RoboClaw(0x80, "/dev/ttyACM1", 9600)	#Port locking still to be done
			break;
		except SerialException:
			rospy.logwarn("Couldn't connect to RoboClaw2. trying again")
			r_time.sleep()
	rospy.loginfo("Connected to RoboClaw2")
	#connected---------------------------------------

	#initialising New_Drive object-------------------
	new_drive = New_Drive(frontClaw,rearClaw)
	#added self.stop in __init__. Add seperately her if it doesnt work
	#------------------------------------------------

	#subscriber lines--------------------------------------------------
	#ros::Subscriber joy_sub = _nh.subscribe("/joy", 100, joyCallback);
	rospy.Subscriber("/gps",gps,new_drive.gps_callback)			#subscriber_id needed
	#rospy.Subscriber("/imu",imu,new_drive.imu_callback)
	#-------------------------------------------------------------------

	#-------------------------------------------------------------------
	#updating the received intructions
	r_time_f=rospy.Rate(10)
	stopped = False
	speed_const = 2		#change this to fine tune
	while not rospy.is_shutdown():
		while(new_drive.initialized == False):
			print ("waiting for first_gps")
			r_time_f.sleep()
		if(stopped == False):
			distance = distance_to_goal(new_drive.posx,new_drive.posy)
			if (distance <= stopping_distance):
				stopped == True
				new_drive.rest = True
			else:	
				new_drive.speed = speed_const * distance
				new_drive.update_steer()
		else:
			print("Arrived within range")	
		#if(new_drive.current_limiter()):			#uncomment after setting current_threshold appropriately
			#print("CURRENT ERROR")
			#stopped = True
			#rospy.loginfo(new_drive.currents)
		rospy.loginfo(new_drive.direction)
		rospy.loginfo(new_drive.speed)
		rospy.loginfo(distance)
		r_time_f.sleep()
	#-------------------------------------------------------------------    
	#left axes forward forward (as on 25th)
	#right axes forward left (as on 25th)
