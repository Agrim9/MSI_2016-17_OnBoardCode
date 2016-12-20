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

ActuatorHigh = 17
ActuatorLow = 27

GPIO.setup(ActuatorHigh,GPIO.OUT)
GPIO.setup(ActuatorLow,GPIO.OUT)


#Analog Pin Setup

VelPin1 = 18


class Arm:
    """docstring for ."""
    def __init__(self, motor_no):
        if((motor_no==1)) :
            self.VelPin=VelPin1
        GPIO.setup(self.VelPin,GPIO.OUT)
        self.actuator_velocity = 0
        self.Velocity=GPIO.PWM(self.VelPin,255)
        self.Velocity.start(0)


    def update(self):
        if self.actuator_velocity < -10:
            self.actuator_backward()
        elif self.actuator_velocity > 10:
            self.actuator_forward()
        else:
            self.actuator_stopped()

    def actuator_forward(self):
        GPIO.output(ActuatorHigh,GPIO.HIGH)
        GPIO.output(ActuatorLow,GPIO.LOW)
        self.Velocity.ChangeDutyCycle(abs(self.actuator_velocity))
        #time.sleep(0.02)

    def actuator_backward(self):
        GPIO.output(ActuatorHigh,GPIO.LOW)
        GPIO.output(ActuatorLow,GPIO.HIGH)
        self.Velocity.ChangeDutyCycle(abs(self.actuator_velocity))
        #time.sleep(0.02)

    def actuator_stopped(self):
        GPIO.output(ActuatorHigh,GPIO.LOW)
        GPIO.output(ActuatorLow,GPIO.LOW)
        self.Velocity.ChangeDutyCycle(0)
        #time.sleep(0.02)
