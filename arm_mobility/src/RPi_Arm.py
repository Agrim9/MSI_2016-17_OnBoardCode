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

Actuator1 = Arm(1)


def drive_callback(inp):
    Actuator1.rover_velocity = inp.data[0]/2.55

if __name__ == "__main__":
    rospy.init_node("rotation_node")
    rospy.loginfo("Starting Actuator Node")
    rospy.Subscriber("/rover/arm_directives", Float64MultiArray, drive_callback)

    while not rospy.is_shutdown():
        p1 = Process(target=Actuator1.update())
        p1.start()
        p1.join()

    GPIO.cleanup()
