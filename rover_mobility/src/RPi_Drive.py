#!/usr/bin/env python
from math import pi, cos, sin

import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

#Digital Pins Setup

FrontHigh = 17
FrontLow = 27
MidHigh = 22
MidLow = 23
BackHigh = 24
BackLow = 25

GPIO.setup(FrontHigh,GPIO.OUT)
GPIO.setup(FrontLow,GPIO.OUT)
GPIO.setup(MidHigh,GPIO.OUT)
GPIO.setup(MidLow,GPIO.OUT)
GPIO.setup(BackHigh,GPIO.OUT)
GPIO.setup(BackLow,GPIO.OUT)


#Analog Pin Setup

VelPin1 = 18
VelPin2 = 12
VelPin3 = 13





class Drive:
    """docstring for ."""
    def __init__(self, motor_no):
        if((motor_no==1)or(motor_no==2)) :
            self.VelPin=VelPin1
        elif((motor_no==3)or(motor_no==4)) :
            self.VelPin=VelPin2
        elif((motor_no==5)or(motor_no==6)) :
            self.VelPin=VelPin3
        GPIO.setup(self.VelPin,GPIO.OUT)
        self.rover_velocity = 0
        self.Velocity=GPIO.PWM(self.VelPin,255)
        self.Velocity.start(0)


    def update(self):
        if self.rover_velocity < 10:
            self.rover_velocity = -self.rover_velocity
            self.all_motor_forward()
        elif self.rover_velocity > 10:
            self.all_motor_backward()
        else:
            self.all_motor_stopped()

    def all_motor_forward(self):
        GPIO.output(FrontHigh,GPIO.HIGH)
        GPIO.output(FrontLow,GPIO.LOW)
        GPIO.output(MidHigh,GPIO.HIGH)
        GPIO.output(MidLow,GPIO.LOW)
        GPIO.output(BackHigh,GPIO.HIGH)
        GPIO.output(BackLow,GPIO.LOW)
        self.Velocity.ChangeDutyCycle(self.rover_velocity)

    def all_motor_backward(self):
        GPIO.output(FrontHigh,GPIO.LOW)
        GPIO.output(FrontLow,GPIO.HIGH)
        GPIO.output(MidHigh,GPIO.LOW)
        GPIO.output(MidLow,GPIO.HIGH)
        GPIO.output(BackHigh,GPIO.LOW)
        GPIO.output(BackLow,GPIO.HIGH)
        self.Velocity.ChangeDutyCycle(self.rover_velocity)

    def all_motor_stopped(self):
        GPIO.output(FrontHigh,GPIO.LOW)
        GPIO.output(FrontLow,GPIO.LOW)
        GPIO.output(MidHigh,GPIO.LOW)
        GPIO.output(MidLow,GPIO.LOW)
        GPIO.output(BackHigh,GPIO.LOW)
        GPIO.output(BackLow,GPIO.LOW)
        self.Velocity.ChangeDutyCycle(0)

def drive_callback(inp):
    Motor1.rover_velocity = inp.data[0]/2.55
    Motor2.rover_velocity = inp.data[1]/2.55
    Motor3.rover_velocity = inp.data[2]/2.55
    Motor4.rover_velocity = inp.data[3]/2.55
    Motor5.rover_velocity = inp.data[4]/2.55
    Motor6.rover_velocity = inp.data[5]/2.55

if __name__ == "__main__":
    rospy.init_node("roboclaw_node")
    rospy.loginfo("Starting Drive Node")
    rospy.Subscriber("/rover/ard_directives", Float64MultiArray, drive_callback)
    Motor1 = Drive(1)
    Motor2 = Drive(2)
    Motor3 = Drive(3)
    Motor4 = Drive(4)
    Motor5 = Drive(5)
    Motor6 = Drive(6)

    while not rospy.is_shutdown():
        Motor1.update()
        Motor2.update()
        Motor3.update()
        Motor4.update()
        Motor5.update()
        Motor6.update()
        rospy.sleep(10)
