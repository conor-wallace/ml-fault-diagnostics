#!/usr/bin/env python
import rospy

import sys
import csv
import geometry_msgs
import std_msgs
import numpy as np
import message_filters
from network_faults.msg import Network, Velocity
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from message_filters import TimeSynchronizer, Subscriber

txrx_pl = 0
txrx_td = 0
roll = 0
pitch = 0
yaw = 0
x = 0
y = 0
z = 0
linear_vel = 0
angular_vel = 0
path_data = []
stop = 0

def gotdata(txrx, vel, pose):
    global txrx_pl,txrx_td, roll, pitch, yaw, x, y, z, linear_vel, angular_vel, path_data, stop

    if not stop:
        if vel != None:
            txrx_pl = txrx.packet_loss
            txrx_td = txrx.time_delay
            linear_vel = vel.lin_vel
            angular_vel = vel.ang_vel
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            quaternion = (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w)
            euler = euler_from_quaternion(quaternion)
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]

            path_data.append([txrx_pl, txrx_td, linear_vel, angular_vel, x, y, z, roll, pitch, yaw])

            print("Packet Loss: %s, Time Delay: %s" % (txrx_pl, txrx_td))
            print("Linear Velocity: %s, Angular Velocity: %s" % (linear_vel, angular_vel))
            print("Roll: %s, Pitch: %s, Yaw: %s" % (roll, pitch, yaw))
            print("X: %s, Y: %s, Z: %s" % (x, y, z))
            print("length of data: %s" % len(path_data))

            if len(path_data) == 200:
                stop = 1
        else:
            print("no messages received yet")

rospy.init_node('GetData', anonymous=True)
network_sub = message_filters.Subscriber("network_stats", Network)
vel_sub = message_filters.Subscriber("lilbot_vel", Velocity)
pose_sub = message_filters.Subscriber("lilbot_3BA615/pose_152", PoseStamped)

ats = message_filters.ApproximateTimeSynchronizer([network_sub, vel_sub, pose_sub],10,0.2)
ats.registerCallback(gotdata)

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    if stop:
        print("converting data to csv")
        path_data = np.asarray(path_data)
        path_data = np.reshape(path_data, [200, 10])
        print(path_data)

        f=open("../data/path_data.csv",'a')
        np.savetxt(f, path_data, delimiter=",")

        sys.exit(1)

    rate.sleep()
