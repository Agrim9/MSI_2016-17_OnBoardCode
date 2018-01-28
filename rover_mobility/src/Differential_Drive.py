#!/usr/bin/env python

from math import pi, cos, sin
import diagnostic_msgs
import diagnostic_updater
import dynamic_reconfigure.client
from roboclaw import RoboClaw
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import spline
from serial.serialutil import SerialException as SerialException

class DifferentialClaw:

    def __init__(self,roboclaw1,roboclaw2):
        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
        0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
        0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
        0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
        0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
        0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
        0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
        0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
        0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
        0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
        0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
        0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
        0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
        0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
        0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
        0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
        0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}

        self.claw1 = roboclaw1
        self.claw2 = roboclaw2
        #self.claw1.ResetEncoders()
        #self.claw2.ResetEncoders()
        self.dir1=1
        self.dir2=1
        self.speed=0
    
    def moveFwd(self):
        print("HEre")
        self.claw1.BackwardM1(min(255,int(self.speed)))
        self.claw1.BackwardM2(min(255,int(self.speed)))    
        self.claw2.ForwardM1(min(255,int(self.speed)))
        self.claw2.BackwardM2(min(255,int(self.speed)))    

    def moveBkwd(self):
        self.claw1.ForwardM1(min(255,int(self.speed)))
        self.claw1.ForwardM2(min(255,int(self.speed)))    
        self.claw2.BackwardM1(min(255,int(self.speed)))
        self.claw2.ForwardM2(min(255,int(self.speed)))    
    
    def moveL(self):
        self.claw1.BackwardM1(min(255,int(self.speed)))
        self.claw1.ForwardM2(min(255,int(self.speed)))
        self.claw2.ForwardM1(min(255,int(self.speed)))
        self.claw2.ForwardM2(min(255,int(self.speed)))

    def moveR(self):
        self.claw1.ForwardM1(min(255,int(self.speed)))
        self.claw1.BackwardM2(min(255,int(self.speed)))    
        self.claw2.BackwardM1(min(255,int(self.speed)))
        self.claw2.BackwardM2(min(255,int(self.speed)))

    def rest(self):
        self.claw1.ForwardM1(0)
        self.claw1.ForwardM2(0)    
        self.claw2.ForwardM1(0)
        self.claw2.ForwardM2(0)   
     
    def update_steer(self):
        if(self.dir1==0 or self.dir2==-0):
            self.rest()        
        elif(self.dir1==1 and self.dir2==1):
            self.moveFwd()
        elif(self.dir1==-1 and self.dir2==-1):
            self.moveBkwd()
        elif(self.dir1==1 and self.dir2==-1):
            self.moveR()
        elif(self.dir1==-1 and self.dir2==1):
            self.moveL()
        
    def steer_callback(self,inp):
            
        if(inp.data[0]>-10 and inp.data[0]<10 and inp.data[6]>-10 and inp.data[6]<10):
            self.speed=0
            self.dir1=0
            self.dir2=0
        
        elif(inp.data[0]>10):
            self.speed=inp.data[0]*inp.data[0]/800
            self.dir1=-1
            self.dir2=-1

        elif(inp.data[0]<-10):
            self.speed=inp.data[0]*inp.data[0]/800
            self.dir1=1
            self.dir2=1
            
        elif(inp.data[6]>10):
            self.speed=inp.data[6]*25*inp.data[6]/800
            self.dir1=1
            self.dir2=-1
            
        elif(inp.data[6]<-10):
            self.speed=inp.data[6]*25*inp.data[6]/800
            self.dir1=-1
            self.dir2=1
            

if __name__ == "__main__":

    rospy.init_node("Differential node")
    rospy.loginfo("Starting differential node")
    r_time=rospy.Rate(1)
    for i in range(20):
        try:
            roboclaw2 = RoboClaw(0x81, "/dev/roboclaw2", 9600)
        except SerialException:
            rospy.logwarn("Could not connect to RoboClaw2, retrying...")
            r_time.sleep()
    rospy.loginfo("Connected to RoboClaw2")

    
    for i in range(20):
        try:
            roboclaw1 = RoboClaw(0x80, "/dev/roboclaw1", 9600)
        except SerialException:
            rospy.logwarn("Could not connect to RoboClaw1, retrying...")
            r_time.sleep()
    rospy.loginfo("Connected to RoboClaw1")

    diffClaw=DifferentialClaw(roboclaw1,roboclaw2)

    diffClaw.claw1.ForwardM1(0)
    diffClaw.claw1.ForwardM2(0)
    diffClaw.claw2.ForwardM1(0)
    diffClaw.claw2.ForwardM2(0)
    diffClaw.speed=0

    rospy.Subscriber("/rover/drive_directives", Float64MultiArray, diffClaw.steer_callback)
    r_time_f=rospy.Rate(10)
    while not rospy.is_shutdown():
        diffClaw.update()
        rospy.loginfo(diffClaw.speed)
        rospy.loginfo(diffClaw.dir1)
        rospy.loginfo(diffClaw.dir2)
        r_time_f.sleep()
