#!/usr/bin/env python
from math import pi, cos, sin

import diagnostic_msgs
import diagnostic_updater
from roboclaw import RoboClaw
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import spline

from serial.serialutil import SerialException as SerialException

time_vec = []
val1_at_t = []
tval1_at_t = []
val2_at_t = []
tval2_at_t = []
plt.ion()
plt.show()
MotorHigh = 17
MotorLow = 27

GPIO.setup(MotorHigh,GPIO.OUT)
GPIO.setup(MotorLow,GPIO.OUT)


#Analog Pin Setup

VelPin1 = 18

class ArmMotor:

    def __init__(self, motor_no , kp1 = 0.10, ki1=0.08, kd1=0.06, int_windout1=50, qpps1 = 5.34, deadzone1 = 30, sample_time=0.1, last_time=0.00, current_time=0.00):

        if((motor_no==1)) :
            self.VelPin=VelPin1
        GPIO.setup(self.VelPin,GPIO.OUT)
        self.actuator_velocity = 0
        self.Velocity=GPIO.PWM(self.VelPin,255)
        self.Velocity.start(0)
        self.targetAngleM1 = 0
		self.targetAngleM2 = 0
		self.kp1 = kp1
		self.ki1 = ki1
		self.kd1 = kd1
		self.qpps1 = qpps1
		self.deadzone1 = deadzone1
		self.int_windout1=int_windout1
		self.kon1 = kon1
		self.sample_time = sample_time
		self.last_time = 0.00
		self.last_error1 = 0.00
		self.PTerm1=0.00
		self.ITerm1=0.00
		self.DTerm1=0.00
		self.delta_error1=0.00
		self.diff1=0.00
		self.enc1Pos=0.00
		self.finalEnc1Val=0.00
		self.lenplt1=0.00

    def update(self):
    	self.current_time = time.time()
        delta_time = self.current_time - self.last_time
		#----------------------------------------------------
		#PID
        if (delta_time >= self.sample_time):
            time_vec.append(self.current_time)
            self.enc1Pos = self.claw.ReadEncM1()[1]
            self.finalEnc1Val = int(self.qpps1*self.targetAngleM1)
            self.diff1 = self.finalEnc1Val - self.enc1Pos  #Error in 1
            self.delta_error1 = self.diff1 - self.last_error1
            self.PTerm1 = self.diff1 #Pterm
            self.ITerm1+=self.diff1*delta_time
            if (self.ITerm1 < -self.int_windout1):
                self.ITerm1 = -self.int_windout1
            elif (self.ITerm1 > self.int_windout1):
                self.ITerm1 = self.int_windout1
            self.DTerm1 = self.delta_error1 / delta_time
            # Remember last time and last error for next calculation
            self.last_error1 = self.diff1

            velM1 = int((self.kp1*self.PTerm1) + (self.ki1 * self.ITerm1) + (self.kd1 * self.DTerm1))

            if self.enc1Pos < (self.finalEnc1Val - self.deadzone1):
                velM1 = velM1 + self.kon1
                val1_at_t.append(self.enc1Pos)
                tval1_at_t.append(self.targetAngleM1)
                self.claw.ForwardM1(min(255, velM1))
            elif self.enc1Pos > (self.finalEnc1Val + self.deadzone1):
                velM1 = velM1 - self.kon1
                val1_at_t.append(self.enc1Pos)
                tval1_at_t.append(self.targetAngleM1)
                self.claw.BackwardM1(min(255, -velM1))
            else:
                self.claw.ForwardM1(0)

    #    rospy.loginfo("%s: %d %d %d %d", self.name, self.diff1, self.targetAngleM1,self.claw.ReadEncM1()[1], velM1)
    #    rospy.loginfo("%s: %d %d %d %d", self.name, self.diff2, self.targetAngleM2,self.claw.ReadEncM2()[1], velM2)



def steer_callback(inp):

	roboclaw2.targetAngleM1 = inp.data[8]
	roboclaw2.targetAngleM2 = inp.data[9]



if __name__ == "__main__":

	rospy.init_node("Motor_node")
	rospy.loginfo("Starting Motor node")

	rospy.Subscriber("/arm/ard_directives", Float64MultiArray, steer_callback)
	rospy.loginfo("I'm here")

	r_time = rospy.Rate(1)

	ArmMotor1 = ArmMotor(1)

	r_time = rospy.Rate(5)


	while not rospy.is_shutdown():
		ArmMotor1.update()
		r_time.sleep()
