#!/usr/bin/env python
from math import pi, cos, sin

import rospy
import tf
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from Class_Drive import Drive
import time
import RPi.GPIO as GPIO
from multiprocessing import Process

Motor1 = Drive(1)
Motor2 = Drive(2)
Motor3 = Drive(3)
Motor4 = Drive(4)
Motor5 = Drive(5)
Motor6 = Drive(6)


def drive_callback(inp):
    Motor1.rover_velocity = inp.data[0]/2.55
    Motor2.rover_velocity = inp.data[1]/2.55
    Motor3.rover_velocity = inp.data[2]/2.55
    Motor4.rover_velocity = inp.data[3]/2.55
    Motor5.rover_velocity = inp.data[4]/2.55
    Motor6.rover_velocity = inp.data[5]/2.55


if __name__ == "__main__":
    rospy.init_node("drive_node")
    rospy.loginfo("Starting Drive Node")
    rospy.Subscriber("/rover/drive_directives", Float64MultiArray, drive_callback)

    while not rospy.is_shutdown():
        Motor1.update()
        Motor2.update()
        Motor3.update()
        Motor4.update()
        Motor5.update()
        Motor6.update()

    GPIO.cleanup()
