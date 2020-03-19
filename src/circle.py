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
    lilbotPub = rospy.Publisher("/lilbot_EFD047/cmd", Int16MultiArray, queue_size=1, tcp_nodelay=True)
    msg = Int16MultiArray()
    rate = rospy.Rate(10.0)
    xv1 = xv2 = xv = wv = 0
    count = 0
    samples = 45
    while not rospy.is_shutdown():

        if count != samples:
            xv1 = 100
            xv2 = 100
            wv = 0

            msg.data = [int(-xv1),int(-xv2),0,int(-wv)]
            lilbotPub.publish(msg)
        else:
            xv1 = 0
            xv2 = 0
            wv = 0

            msg.data = [int(-xv1),int(-xv2),0,int(-wv)]
            lilbotPub.publish(msg)
            sys.exit(1)

        count += 1
        rate.sleep()
