#!/usr/bin/env python

from roboclaw import RoboClaw
import rospy
import tf
from std_msgs.msg import Float64MultiArray
import numpy as np
import signal
import sys
from serial.serialutil import SerialException as SerialException

#-----------------------------------------------------------------
#SIGINT handler
def sigint_handler(signal, frame):
    sys.exit(0)

class DifferentialClaw:

    def __init__(self,roboclaw1,roboclaw2):
        self.frontClaw = roboclaw1
        self.rearClaw = roboclaw2
        self.dir1=1
        self.dir2=1
        self.speed=0
    
    def moveFwd(self):
        
        self.frontClaw.ForwardM1(min(255,int(self.speed)))
        self.frontClaw.ForwardM2(min(255,int(self.speed)))    
        self.rearClaw.ForwardM1(min(255,int(self.speed)))
        self.rearClaw.ForwardM2(min(255,int(self.speed)))    

    def moveBkwd(self):
        self.frontClaw.BackwardM1(min(255,int(self.speed)))
        self.frontClaw.BackwardM2(min(255,int(self.speed)))    
        self.rearClaw.BackwardM1(min(255,int(self.speed)))
        self.rearClaw.BackwardM2(min(255,int(self.speed)))    
    
    def moveL(self):
        self.frontClaw.BackwardM1(min(255,int(self.speed)))
        self.frontClaw.ForwardM2(min(255,int(self.speed)))
        self.rearClaw.BackwardM1(min(255,int(self.speed)))
        self.rearClaw.ForwardM2(min(255,int(self.speed)))

    def moveR(self):
        self.frontClaw.ForwardM1(min(255,int(self.speed)))
        self.frontClaw.BackwardM2(min(255,int(self.speed)))    
        self.rearClaw.ForwardM1(min(255,int(self.speed)))
        self.rearClaw.BackwardM2(min(255,int(self.speed)))

    def rest(self):
        self.frontClaw.ForwardM1(0)
        self.frontClaw.ForwardM2(0)    
        self.rearClaw.ForwardM1(0)
        self.rearClaw.ForwardM2(0)   
     
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
            print("Fwd Commanded")
            self.dir1=1
            self.dir2=1

        elif(inp.data[0]<-10):
            self.speed=inp.data[0]*inp.data[0]/800
            print("Bkwd Commanded")
            self.dir1=-1
            self.dir2=-1
            
        elif(inp.data[6]>10):
            self.speed=inp.data[6]*25*inp.data[6]/800
            print("Right Commanded")
            self.dir1=1
            self.dir2=-1
            
        elif(inp.data[6]<-10):
            print("Left Commanded")
            self.speed=inp.data[6]*25*inp.data[6]/800
            self.dir1=-1
            self.dir2=1
            

if __name__ == "__main__":

    signal.signal(signal.SIGINT, sigint_handler)
    rospy.init_node("Differential node")
    rospy.loginfo("Starting differential node")
    r_time=rospy.Rate(1)
    for i in range(20):
        try:
            rearClaw = RoboClaw(0x81, "/dev/rearClaw", 9600)
        except SerialException:
            rospy.logwarn("Could not connect to RoboClaw2, retrying...")
            r_time.sleep()
    rospy.loginfo("Connected to RoboClaw2")

    
    for i in range(20):
        try:
            frontClaw = RoboClaw(0x80, "/dev/frontClaw", 9600)
        except SerialException:
            rospy.logwarn("Could not connect to RoboClaw1, retrying...")
            r_time.sleep()
    rospy.loginfo("Connected to RoboClaw1")

    diffClaw=DifferentialClaw(frontClaw,rearClaw)

    diffClaw.frontClaw.ForwardM1(0)
    diffClaw.frontClaw.ForwardM2(0)
    diffClaw.rearClaw.ForwardM1(0)
    diffClaw.rearClaw.ForwardM2(0)
    diffClaw.speed=0

    rospy.Subscriber("/rover/drive_directives", Float64MultiArray, diffClaw.steer_callback)
    r_time_f=rospy.Rate(10)
    while not rospy.is_shutdown():
        diffClaw.update_steer()
        #rospy.loginfo(diffClaw.speed)
        #rospy.loginfo(diffClaw.dir1)
        #rospy.loginfo(diffClaw.dir2)
        r_time_f.sleep()
