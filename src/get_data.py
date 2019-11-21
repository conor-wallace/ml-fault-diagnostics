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
msg_data = []
stop = 0

def gotdata(txrx, imu, pose):
    global stop, msg_data

    if not stop:
        quaternion = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        msg_data.append([txrx.time_delay, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, roll, pitch, yaw, imu.roll, imu.pitch, imu.yaw, imu.accel_y, imu.accel_z, imu.lin_x, imu.lin_y, imu.lin_z])

        if len(msg_data) == 200:
            stop = 1

rospy.init_node('GetData', anonymous=True)
network_sub = message_filters.Subscriber("network_stats", Network)
pose_sub = message_filters.Subscriber("lilbot_6ADE87/pose_152", PoseStamped)
imu_sub = message_filters.Subscriber("/imu", IMU)

ats = message_filters.ApproximateTimeSynchronizer([network_sub, imu_sub, pose_sub],10,0.2)
ats.registerCallback(gotdata)

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    if stop:
        print("converting data to csv")
        msg_data = np.asarray(msg_data)
        msg_data = np.reshape(msg_data, [200, 15])

        f=open("../data/path_data.csv",'a')
        np.savetxt(f, path_data, delimiter=",")

        sys.exit(1)

    rate.sleep()
