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
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import time
import matplotlib.pyplot as plt
import numpy as np
import thread
from scipy.interpolate import spline

from serial.serialutil import SerialException as SerialException

plt.ion()
ax1=plt.axes()
ax1.set_ylim(0,2048)
enc1plotr, = plt.plot([0]*50,color='blue')
ip_enc1plotr, = plt.plot([0]*50,color='green',linestyle='dotted')
class SteerClaw:

    def __init__(self, address, dev_name, baud_rate, name, kp1 = 3, kp2=3,ki1=40,ki2=40,kii1=80,kii2=80,kd1=2,kd2=2,iint_windout1=400,iint_windout2=400,int_windout1=200,int_windout2=200, deadzone1 = 2, deadzone2 = 2, sample_time=0.1, last_time=0.00, current_time=0.00):
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
        self.kii1=kii1
        self.kii2=kii2
        self.iint_windout1=iint_windout1
        self.iint_windout2=iint_windout2
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
        self.IIterm1=0.00
        self.IIterm2=0.00
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
        self.lastIIerr1=0.00
        self.lastIIerr2=0.00
        self.diffIIterm1=0.00
        self.diffIIterm2=0.00
        self.enc1PosData=[0]*50
        self.ip_enc1PosDatar=[0]*50

    def pub_pot(self, pub, claw_name):
        pot_val1 = self.claw.ReadEncM1()[1]
        pot_val2 = self.claw.ReadEncM2()[1]
        pub.publish(claw_name+"| Pot1 Val :" +str(pot_val1) + "| Pot2 Val :" +str(pot_val2))

    def pub_curr(self,pub,claw_name):
        curr_val = self.claw.ReadCurrents()
        pub.publish(claw_name+"| Curr1 Val :" +str(curr_val[1]) + "| Curr2 Val :" +str(curr_val[2]))

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
                if(self.lastIIerr1!=0):
                    self.diffIIterm1=self.ITerm1-self.lastIIerr1
                    self.IIterm1+=self.diffIIterm1*delta_time
                if (self.IIterm1 < -self.iint_windout1):
                    self.IIterm1 = -self.iint_windout1
                elif (self.IIterm1 > self.iint_windout1):
                    self.IIterm1 = self.iint_windout1
                
                if (self.ITerm1 < -self.int_windout1):
                    self.ITerm1 = -self.int_windout1
                elif (self.ITerm1 > self.int_windout1):
                    self.ITerm1 = self.int_windout1
                self.DTerm1 = self.delta_error1 / delta_time
                 # Remember last time and last error for next calculation
                self.last_error1 = self.diff1
                self.last_Rpm1 = self.current_Rpm1
                self.last_enc1pos = self.enc1Pos
                self.lastIIerr1=self.ITerm1
                velM1 = int((self.kp1*self.PTerm1) + (self.ki1 * self.ITerm1) + (self.kd1 * self.DTerm1)+(self.kii1*self.IIterm1))
                self.enc1PosData.append(velM1)
                self.ip_enc1PosDatar.append(self.targetRpm1)
                del self.ip_enc1PosDatar[0]
                del self.enc1PosData[0]
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
                if(self.lastIIerr2!=0):
                    self.diffIIterm2=self.ITerm2-self.lastIIerr2
                    self.IIterm2+=self.diffIIterm2*delta_time
                if (self.IIterm2 < -self.iint_windout2):
                    self.IIterm2 = -self.iint_windout2
                elif (self.IIterm2 > self.iint_windout2):
                    self.IIterm2 = self.iint_windout2
        
                if (self.ITerm2 < -self.int_windout2):
                    self.ITerm2 = -self.int_windout2
                elif (self.ITerm2 > self.int_windout2):
                    self.ITerm2 = self.int_windout2

                self.DTerm1 = self.delta_error2 / delta_time
                # Remember last time and last error for next calculation
                self.last_error2 = self.diff2
                self.last_Rpm2 = self.current_Rpm2
                self.last_enc2pos = self.enc2Pos
                self.lastIIerr2=self.ITerm2
                velM2 = int((self.kp2*self.PTerm2) + (self.ki2 * self.ITerm2) + (self.kd2 * self.DTerm2)+(self.kii2*self.IIterm2))
                if self.current_Rpm2 < (self.finalRpm2Val - self.deadzone2):
                    self.claw.ForwardM2(min(125, velM2))
                elif self.current_Rpm2 > (self.finalRpm2Val + self.deadzone2):
                    self.claw.BackwardM2(min(125, -velM2))
                else:
                    self.claw.ForwardM2(0)



        #----------------------------------------------------


    #    rospy.loginfo("%s: %d %d %d %d", self.name, self.diff1, self.targetAngleM1,self.claw.ReadEncM1()[1], velM1)
    #    rospy.loginfo("%s: %d %d %d %d", self.name, self.diff2, self.targetAngleM2,self.claw.ReadEncM2()[1], velM2)



def steer_callback(inp):

    actuator_lock = inp.data[1]
    
    if actuator_lock == 1:
        roboclaw1.targetRpm1 = 230
    elif actuator_lock == -1:
        roboclaw1.targetRpm1 = -230
    else: 
        roboclaw1.targetRpm1 = 0

    elbowmotor_lock = inp.data[2]
    
    if elbowmotor_lock == 1:
        roboclaw1.targetRpm2 = 230
    elif elbowmotor_lock == -1:
        roboclaw1.targetRpm2 = -230
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

def reconfig_callback(inp):
    roboclaw1.kp1=inp.data[0]
    roboclaw1.kp2=inp.data[0]
    roboclaw1.ki1=inp.data[1]
    roboclaw1.ki2=inp.data[1]
    roboclaw1.kd1=inp.data[2]
    roboclaw1.kd2=inp.data[2]
    roboclaw1.kii1=inp.data[3]
    roboclaw1.kii2=inp.data[3]
    rospy.loginfo("Reconfig Acknowledged : ")
    rospy.loginfo(inp)

if __name__ == "__main__":

    rospy.init_node("roboclaw_node")
    rospy.loginfo("Starting steer node")
    pub = rospy.Publisher('Pot_Val', String, queue_size=10)
    pub1 = rospy.Publisher('Curr_Val', String, queue_size=10)
    rospy.Subscriber("/rover/arm_directives", Float64MultiArray, steer_callback)
    rospy.Subscriber("/arm_conf_mssg",Float32MultiArray,reconfig_callback)
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
            roboclaw1 = SteerClaw(0x81, "/dev/ttyACM0", 9600, "BaseClaw")
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
        enc1plotr.set_xdata(np.arange(len(roboclaw1.enc1PosData)))
        enc1plotr.set_ydata(roboclaw1.enc1PosData)  # update the data
        ip_enc1plotr.set_xdata(np.arange(len(roboclaw1.ip_enc1PosDatar)))
        ip_enc1plotr.set_ydata(roboclaw1.ip_enc1PosDatar)  # update the data
        plt.draw()
        roboclaw1.pub_pot(pub,"roboclaw1")
        r_time.sleep()

    roboclaw1.claw.ForwardM1(0)
    roboclaw1.claw.ForwardM2(0)
    #roboclaw2.claw.ForwardM1(0)
    #roboclaw2.claw.ForwardM2(0)
