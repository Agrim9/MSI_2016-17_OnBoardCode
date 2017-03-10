#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from arm_mobility.cfg import armGainConfig
from std_msgs.msg import Float32MultiArray

outMsg = Float32MultiArray()
def callback(config, level):
    rospy.loginfo("""Reconfigure Request: Kp {Kp} Ki {Ki} Kd {Kd} Kii {Kii} """.format(**config))
    data =[config.Kp,config.Ki, config.Kd, config.Kii]
    rospy.loginfo("Gains Changed")
    rospy.loginfo(data)
    outMsg.data = data;
    pub.publish(outMsg)
    return config

if __name__ == "__main__":
    rospy.init_node("arm_dyn_server", anonymous = True)
    pub = rospy.Publisher('arm_conf_mssg', Float32MultiArray, queue_size=10)
    srv = Server(armGainConfig, callback)
    rospy.spin()