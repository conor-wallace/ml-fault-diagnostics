#!/usr/bin/env python
import rospy
import sys
import math
import tf2_ros
import tf_conversions
import geometry_msgs.msg
import turtlesim.srv
import numpy as np
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
from network_faults.msg import Velocity
import matplotlib.pyplot as plt

if __name__ == '__main__':

    rospy.init_node('lilbot_controller_tf2_listener')
    lilbotPub = rospy.Publisher("/lilbot_6ADE87/cmd", Int16MultiArray, queue_size=1, tcp_nodelay=True)
    msg = Int16MultiArray()
    rate = rospy.Rate(10.0)
    xv1 = xv2 = xv = wv = 0
    while not rospy.is_shutdown():
        xv1 = 175
        xv2 = 175
        wv = 250

        msg.data = [int(-xv1),int(-xv2),0,int(-wv)]
        lilbotPub.publish(msg)

        rate.sleep()
