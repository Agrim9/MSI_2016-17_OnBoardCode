#!/usr/bin/env python
#Contains codes for all drive related things. Imported in other codes for usage. 

#Edited on 28/09/19 by Kavin and Preetam

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
# import RPi.GPIO as g
import time
#---------------------------------------------------- 
gps_dest = [71, 19]
dist_threshold = 5
angle_threshold = 5
c1 = 0.5*dist_threshold
c2 = 0.5*angle_threshold

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
        self.mode_oldval="front"
        self.gps_curr = gps_dest
        self.current_heading = 0
        self.currents = [0,0,0,0]
        self.current_threshold = 400

    def fwd(self):
        self.rightClaw.ForwardM1(self.speed)
        self.rightClaw.BackwardM2(self.speed)
        self.leftClaw.BackwardM1(self.speed)
        self.leftClaw.ForwardM2(self.speed)
        # self.rightClaw.BackwardM1(self.speed)
        # self.rightClaw.ForwardM2(self.speed)
        # self.leftClaw.BackwardM1(self.speed)
        # self.leftClaw.ForwardM2(self.speed)

    def bwd(self):
        self.rightClaw.BackwardM1(self.speed)
        self.rightClaw.ForwardM2(self.speed)
        self.leftClaw.ForwardM1(self.speed)
        self.leftClaw.BackwardM2(self.speed)
        # self.rightClaw.ForwardM1(self.speed)
        # self.rightClaw.BackwardM2(self.speed)
        # self.leftClaw.ForwardM1(self.speed)
        # self.leftClaw.BackwardM2(self.speed)

    def stop(self):
        self.rightClaw.ForwardM1(0)
        self.leftClaw.ForwardM1(0)
        self.rightClaw.ForwardM2(0)
        self.leftClaw.ForwardM2(0)

    def left(self):
        self.rightClaw.ForwardM1(self.speed)
        self.rightClaw.BackwardM2(self.speed)
        self.leftClaw.ForwardM1(self.speed)
        self.leftClaw.BackwardM2(self.speed)
        
    def right(self):
        self.rightClaw.BackwardM1(self.speed)
        self.rightClaw.ForwardM2(self.speed)
        self.leftClaw.BackwardM1(self.speed)
        self.leftClaw.ForwardM2(self.speed)
            
    def update_steer(self):
        # if(not(self.rest)):
            # print(self.rest,self.direction)
        if(self.rest == True):
            self.stop()
        elif(self.direction == "forward"):
            self.fwd()
        elif(self.direction == "backward"):
            self.bwd()
        elif(self.direction == "right"):
            self.right()
        else:
            self.left()

    # def update_turn(self):
    #     if(self.turn == False):
    #         self.stop()
    #     elif(self.turn_dir == "left"):
    #         self.left()
    #     else:
    #         self.right()                

    def drive_callback(self,inp):
        axes = inp.axes
        buttons = inp.buttons
        if(buttons[6] == 1): #Find which button
            if(self.drivemode == "open_loop"):
                # self.drivemode == "autonomous_mode"
                self.drivemode == "open_loop"
            else:
                self.drivemode == "open_loop"

        if(self.drivemode == "open_loop"):
            # print(axes[1],axes[3])
            if(axes[1]<0.1 and axes[1]>-(0.1) and axes[3]<0.1 and axes[3]>-0.1):
                self.speed = 0
                self.rest = True
            elif (axes[1]>0.1 or axes[1]<-0.1):
                self.rest = False
                self.speed = int(min(255,abs(400*axes[1]*axes[1])))  
                if(axes[1] > 0):
                    self.direction = "forward"
                else:   
                    self.direction = "backward"
            else:
                self.rest = False
                self.speed = int(min(255,abs(400*axes[3]*axes[3])))
                if(axes[3] > 0):
                    self.direction = "left"
                else:
                    self.direction = "right"
            # print(self.speed)

            # if(buttons[7]==1):
            #     if(self.mode_oldval=="front"):
            #         self.mode_oldval="all"
            #     else:
            #         self.mode_oldval="front"
        else:
            self.autonomous_move(gps_dest)            

    
    def autonomous_move(self, gps_dest):
        distance = sqrt((to_utm(gps_dest)[0] - to_utm(self.gps_curr)[0])**2 + (to_utm(gps_dest)[1] - to_utm(self.gps_curr)[1])**2)    
        while(distance > dist_threshold):
            dx = to_utm(gps_dest)[0] - to_utm(self.gps_curr)[0]
            dy = to_tm(gps_dest)[1] - to_utm(self.gps_curr)[1]
            target_angle = 90 - np.arctan(dy/dx)
            angle_diff = current_heading - target_angle
            auto_fwd_speed = 255*(1 - exp(-(distance - c1)))
            if(abs(angle_diff) > angle_threshold):
                self.turn = True
                self.rest = True
                auto_rot_speed = 255*(1 - exp(-(angle_diff - c2)))
                self.speed = int(min(255,auto_rot_speed))
                if(angle_diff > 0): self.turn_dir == "right"
                else: self.turn_dir == "left"
            else:
                self.turn = False
                self.rest = False
                self.speed = int(min(255,auto_fwd_speed))
                self.direction == "forward"
        return          

    def GPS(self, inp):
        self.gps_curr = [inp[0], inp[1]]
        return
 
    def to_utm(gps):
        return utm.from_latlon(gps[0], gps[1])

    def IMU(self, inp):
        self.current_heading = inp #Check IMU data format
        return

    # def autonom_ahead(self,speed_auto):
    #     auto_speed = int(min(255,speed_auto))
    #     self.rightClaw.ForwardM1(auto_speed)
    #     self.rightClaw.BackwardM2(auto_speed)
    #     self.leftClaw.BackwardM1(auto_speed)
    #     self.leftClaw.ForwardM2(auto_speed)
        
    # def autonom_turn(self,a,c):
    #     g.output(23,c)
    #     g.output(4,c)
    #     self.pwm4.ChangeDutyCycle(50)
    #     self.pwm1.ChangeDutyCycle(50)
    #     time.sleep(a)
    #     self.pwm4.ChangeDutyCycle(0)
    #     self.pwm1.ChangeDutyCycle(0)


    # def diff_turn(dir):
    #     g.output(23,dir)
    #     g.output(4,dir)
    #     g.output(22,not dir)
    #     g.output(24,not dir)
    #     pwm1.ChangeDutyCycle(50)
    #     pwm2.ChangeDutyCycle(50)
    #     pwm3.ChangeDutyCycle(0)
    #     pwm4.ChangeDutyCycle(0)
    #     time.sleep(4.5)
    #     pwm1.ChangeDutyCycle(0)
    #     pwm2.ChangeDutyCycle(0)
    #     pwm3.ChangeDutyCycle(0)
    #     pwm4.ChangeDutyCycle(0)

    def current_limiter(self):
        (i,self.currents[0],self.currents[1]) = self.rightClaw.ReadCurrents()
        (i,self.currents[2],self.currents[3]) = self.leftClaw.ReadCurrents()
        for i in range(4):
            if(self.currents[i] > self.current_threshold):
                self.stop()
                return True
        return False

    def update_current(self):        
        (i,self.currents[0],self.currents[1]) = self.rightClaw.ReadCurrents()
        (i,self.currents[2],self.currents[3]) = self.leftClaw.ReadCurrents()
#---------------------------------------------------                



