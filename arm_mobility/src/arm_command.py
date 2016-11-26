#!/usr/bin/env python
from math import pi, cos, sin

import diagnostic_msgs
import diagnostic_updater
import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import time
import matplotlib.pyplot as plt
import numpy as np
import RPi.GPIO as GPIO
from scipy.interpolate import spline
from std_msgs.msg import String

from serial.serialutil import SerialException as SerialException

time_vec = []
val1_at_t = []
tval1_at_t = []
val2_at_t = []
tval2_at_t = []
plt.ion()
plt.show()
VelPin1 = 18
MotorHigh = 17
MotorLow = 27
SPI_CLK=18
SPI_MISO=23
SPI_MOSI=24
SPI_CS=25
GPIO.setmode(GPIO.BCM)
GPIO.setup(MotorHigh,GPIO.OUT)
GPIO.setup(MotorLow,GPIO.OUT)
GPIO.setup(SPI_MOSI, GPIO.OUT)
GPIO.setup(SPI_MISO, GPIO.IN)
GPIO.setup(SPI_CLK, GPIO.OUT)
GPIO.setup(SPI_CS, GPIO.OUT)

#10k pot connected to adc #1
adc_used=0

tolerance=2 #we change readings only when pot moves 5 counts

#Analog Pin Setup


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
        #self.kon1 = kon1
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
        self.kon1=0.00

    def update(self):
    	self.current_time = time.time()
        delta_time = self.current_time - self.last_time
		#----------------------------------------------------
		#PID
        if (delta_time >= self.sample_time):
            time_vec.append(self.current_time)
            #self.enc1Pos = self.claw.ReadEncM1()[1]
            self.enc1Pos = 0
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
                #self.claw.ForwardM1(min(255, velM1))
            elif self.enc1Pos > (self.finalEnc1Val + self.deadzone1):
                velM1 = velM1 - self.kon1
                val1_at_t.append(self.enc1Pos)
                tval1_at_t.append(self.targetAngleM1)
                #self.claw.BackwardM1(min(255, -velM1))
            #else:
                #self.claw.ForwardM1(0)

    #    rospy.loginfo("%s: %d %d %d %d", self.name, self.diff1, self.targetAngleM1,self.claw.ReadEncM1()[1], velM1)
    #    rospy.loginfo("%s: %d %d %d %d", self.name, self.diff2, self.targetAngleM2,self.claw.ReadEncM2()[1], velM2)


#Reading Analog data from ADC MCP3008, having 8 possible ADC's ( 0 to 7)
# ADC_norepresents the ADC we are reading data from
def Read_ADC(adc_no, clk, mosi, miso, cs):
    if((adc_no>7) or (adc_no<0)): 
        return -1
    GPIO.output(cs, True) #CS bar, chip select should be high initially

    GPIO.output(clk, False) #start clock low
    GPIO.output(cs, False) #get CS bar to low now, selecting the ADC for usage

    ADC_sel = adc_no
    ADC_sel |= 0x18 #start_bit high(00011000) 
    ADC_sel <<= 3

    ##### Select proper ADC out of 8 #####
    for i in range(5):
        if (ADC_sel & 0x80):
            GPIO.output(mosi,True)
        else:
            GPIO.output(mosi,False)
        ADC_sel<<=1
        GPIO.output(clk, True)
        GPIO.output(clk, True)
    
    ADC_out=0
    #ADC now selected, read value from it
    #read one empty bit, one null bit and 10 ADC bits
    for i in range(12):
        GPIO.output(clk, True)
        GPIO.output(clk, False)
        ADC_out<<=1
        if(GPIO.input(miso)):
            ADC_out |= 0x01
    GPIO.output(cs, True) #Deselect the chip by stating CS bar as high
    ADC_out >>=1 #First bit is null, drop it
    return ADC_out

if __name__ == "__main__":

    rospy.init_node("Motor_node")
    rospy.loginfo("Starting Motor node")
    pub = rospy.Publisher('ADC_Val', String, queue_size=10)
    rospy.loginfo("I'm here")

    r_time = rospy.Rate(1)

    ArmMotor1 = ArmMotor(1)

    r_time = rospy.Rate(5)

    old_val=0

    while not rospy.is_shutdown():
        
        pot_changed=False #initial assumption
        new_val=Read_ADC(adc_used, SPI_CLK, SPI_MOSI, SPI_MISO, SPI_CS)
        diff_val=new_val-old_val
        pub.publish("Pot Val : " +str(new_val))
        if(diff_val>tolerance):
            pot_changed=True
        if(pot_changed):
            ArmMotor1.targetAngleM1=new_val*360/31 #Mapping 360 to 3.3 voltage
            ArmMotor1.update()
            #pub.publish("Set Target Angle to " + str(ArmMotor1.targetAngleM1))
            rospy.loginfo("Set Target Angle to " + str(ArmMotor1.targetAngleM1))
        old_val=new_val
        r_time.sleep()
GPIO.cleanup()
