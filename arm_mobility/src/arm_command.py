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
from std_msgs.msg import String
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import spline

from serial.serialutil import SerialException as SerialException

class SteerClaw:

    def __init__(self, address, dev_name, baud_rate, name, kp1 = 0.15, kp2=0.15,ki1=0.15,ki2=0.15,kd1=0.8,kd2=0.8,int_windout1=50,int_windout2=50, qpps1 = 5.34, qpps2 = 5.34, deadzone1 = 30, deadzone2 = 20, sample_time=0.1, last_time=0.00, current_time=0.00):
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

        self.claw = RoboClaw(address, dev_name, baud_rate)
        self.name = name
        self.claw.ResetEncoders()
        self.targetRpm1=0
        self.targetRpm2=0
        self.kp1 = kp1
        self.kp2 = kp2
        self.ki1 = ki1
        self.ki2 = ki2
        self.kd1 = kd1
        self.kd2 = kd2
        self.qpps1 = qpps1
        self.qpps2 = qpps2
        self.deadzone1 = deadzone1
        self.deadzone2 = deadzone2
        self.int_windout1=int_windout1
        self.int_windout2=int_windout2
        self.sample_time = sample_time
        self.last_time = 0.00
        self.last_error1 = 0.00
        self.last_error2 = 0.00
        self.PTerm1=0.00
        self.ITerm1=0.00
        self.DTerm1=0.00
        self.PTerm2=0.00
        self.ITerm2=0.00
        self.DTerm2=0.00
        self.delta_error1=0.00
        self.delta_error2=0.00
        self.diff1=0.00
        self.diff2=0.00
        self.enc1Pos=0.00
        self.enc2Pos=0.00
        self.finalRpm1Val=0.00
        self.finalRpm2Val=0.00
        self.lenplt1=0.00
        self.lenplt2=0.00
        self.last_Rpm1=0.00
        self.last_Rpm2=0.00
        self.last_enc1pos = 0.00
        self.last_enc2pos = 0.00

    def pub_pot(self, pub, claw_name):
        pot_val1 = self.claw.ReadEncM1()
        pot_val2 = self.claw.ReadEncM2()
        pub.publish(claw_name+"| Pot1 Val :" +str(pot_val1) + "| Pot2 Val :" +str(pot_val2))

    def update_rpm_1(self):
            self.current_time = time.time()
            delta_time = self.current_time - self.last_time
            if (delta_time >= self.sample_time):
                self.enc1Pos = self.claw.ReadEncM1()[1]
                self.current_Rpm1 = (self.enc1Pos - self.last_enc1pos)/(delta_time)
                self.finalRpm1Val = int(self.targetRpm1)
                self.diff1 = self.finalRpm1Val - self.current_Rpm1  #Error in 1
                self.delta_error1 = self.diff1 - self.last_Rpm1
                self.PTerm1 = self.diff1 #Pterm
                self.ITerm1+=self.diff1*delta_time
                if (self.ITerm1 < -self.int_windout1):
                    self.ITerm1 = -self.int_windout1
                elif (self.ITerm1 > self.int_windout1):
                    self.ITerm1 = self.int_windout1
                self.DTerm1 = self.delta_error1 / delta_time
                 # Remember last time and last error for next calculation
                self.last_error1 = self.diff1
                self.last_Rpm1 = self.current_Rpm1
                self.last_enc1pos = self.enc1Pos

                velM1 = int((self.kp1*self.PTerm1) + (self.ki1 * self.ITerm1) + (self.kd1 * self.DTerm1))

                if self.current_Rpm1 < (self.finalRpm1Val - self.deadzone1):
                    self.claw.ForwardM1(min(255, velM1))
                elif self.current_Rpm1 > (self.finalRpm1Val + self.deadzone1):
                    self.claw.BackwardM1(min(255, -velM1))
                else:
                    self.claw.ForwardM1(0)

    def update_rpm_2(self):
            self.current_time = time.time()
            delta_time = self.current_time - self.last_time
            if (delta_time >= self.sample_time):
                self.enc2Pos = -self.claw.ReadEncM2()[1]
                self.current_Rpm2 = (self.enc2Pos - self.last_enc2pos)/(delta_time)
                self.finalRpm2Val = int(self.targetRpm2)
                self.diff2 = self.finalRpm2Val - self.current_Rpm2
                self.delta_error2 = self.diff2 - self.last_Rpm2
                self.PTerm2 = self.diff2 #Pterm
                self.ITerm2+=self.diff2*delta_time
                if (self.ITerm2 < -self.int_windout2):
                    self.ITerm2 = -self.int_windout2
                elif (self.ITerm2 > self.int_windout2):
                    self.ITerm2 = self.int_windout2

                self.DTerm1 = self.delta_error2 / delta_time
                # Remember last time and last error for next calculation
                self.last_error2 = self.diff2
                self.last_Rpm2 = self.current_Rpm2
                self.last_enc2pos = self.enc2Pos
                velM2 = int((self.kp2*self.PTerm2) + (self.ki2 * self.ITerm2) + (self.kd2 * self.DTerm2))
                if self.current_Rpm2 < (self.finalRpm2Val - self.deadzone2):
                    self.claw.ForwardM2(min(255, velM2))
                elif self.current_Rpm2 > (self.finalRpm2Val + self.deadzone2):
                    self.claw.BackwardM2(min(255, -velM2))
                else:
                    self.claw.ForwardM2(0)



        #----------------------------------------------------


    #    rospy.loginfo("%s: %d %d %d %d", self.name, self.diff1, self.targetAngleM1,self.claw.ReadEncM1()[1], velM1)
    #    rospy.loginfo("%s: %d %d %d %d", self.name, self.diff2, self.targetAngleM2,self.claw.ReadEncM2()[1], velM2)



def steer_callback(inp):

    actuator_lock = inp.data[1]
    
    if actuator_lock == 1:
        roboclaw1.targetRpm1 = 150
    elif actuator_lock == -1:
        roboclaw1.targetRpm1 = -150
    else: 
        roboclaw1.targetRpm1 = 0

    elbowmotor_lock = inp.data[2]
    
    if elbowmotor_lock == 1:
        roboclaw1.targetRpm2 = 150
    elif elbowmotor_lock == -1:
        roboclaw1.targetRpm2 = -150
    else:
        roboclaw1.targetRpm2 = 0
    
    pitchmotor_lock = inp.data[3]
    
    #if pitchmotor_lock == 1:
    #    roboclaw2.targetRpm1 = 150
    #elif pitchmotor_lock == -1:
    #    roboclaw2.targetRpm1 = -150
    #else: 
    #    roboclaw2.targetRpm1 = 0

    grippermotor_lock = inp.data[5]

    #if grippermotor_lock == 1:
    #    roboclaw2.targetRpm2 = 150
    #elif grippermotor_lock == -1:
    #    roboclaw2.targetRpm2 = -150
    #else: 
    #    roboclaw2.targetRpm2 = 0


if __name__ == "__main__":

    rospy.init_node("roboclaw_node")
    rospy.loginfo("Starting steer node")
    pub = rospy.Publisher('Pot_Val', String, queue_size=10)
    rospy.Subscriber("/rover/arm_directives", Float64MultiArray, steer_callback)
    
    r_time = rospy.Rate(1)

    #for i in range(20):
    #       try:
    #               roboclaw2 = SteerClaw(0x81, "/dev/roboclaw_a2", 9600, "GripClaw")
    #       except SerialException:
    #               rospy.logwarn("Could not connect to Arm RoboClaw2, retrying...")
    #               r_time.sleep()
    #rospy.loginfo("Connected to Arm RoboClaw2")
    

    for i in range(20):
        try:
            roboclaw1 = SteerClaw(0x80, "/dev/roboclaw_a1", 9600, "BaseClaw")
        except SerialException:
            rospy.logwarn("Could not connect to Arm RoboClaw1, retrying...")
            r_time.sleep()
    rospy.loginfo("Connected to Arm RoboClaw1")


    r_time = rospy.Rate(5)
    roboclaw1.claw.ForwardM1(0)
    roboclaw1.claw.ForwardM2(0)
    #roboclaw2.claw.ForwardM1(0)
    #roboclaw2.claw.ForwardM2(0)

    while not rospy.is_shutdown():
        roboclaw1.update_rpm_1()
        roboclaw1.update_rpm_2()
        #roboclaw2.update_rpm_1()
        #roboclaw2.update_rpm_2()
        r_time.sleep()

    roboclaw1.claw.ForwardM1(0)
    roboclaw1.claw.ForwardM2(0)
    #roboclaw2.claw.ForwardM1(0)
    #roboclaw2.claw.ForwardM2(0)
