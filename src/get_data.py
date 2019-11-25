#!/usr/bin/env python
import rospy

import sys
import csv
import geometry_msgs
import std_msgs
import numpy as np
import message_filters
from network_faults.msg import Network, Velocity, IMU
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from message_filters import TimeSynchronizer, Subscriber

# TODO: change /dev/ttyUSB0 to /dev/ttyAMA0 in serial_data.py and utils.py in littlebot_155

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
offset = []
stop = 0
first_read = 1

def gotDataCallback(imu):
    global stop, msg_data, offset
    print(len(msg_data))

    if first_read:
        offset.append([imu.pitch, imu.yaw, imu.accel_y, imu.accel_z, imu.lin_x, imu.lin_y, imu.lin_z])
        first_read = 0

    if not stop:
        msg_data.append([imu.pitch-offset[0], imu.yaw-offset[1], imu.accel_y-offset[2], imu.accel_z-offset[3], imu.lin_x-offset[4], imu.lin_y-offset[5], imu.lin_z-offset[6]])

        if len(msg_data) == 200:
            stop = 1

rospy.init_node('GetData', anonymous=True)
rospy.Subscriber("/imu", IMU, gotDataCallback)
rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    if stop:
        print("converting data to csv")
        msg_data = np.asarray(msg_data)
        msg_data = np.reshape(msg_data, [200, 7])

        f=open("../data/path_data.csv",'a')
        np.savetxt(f, msg_data, delimiter=",")

        sys.exit(1)

    rate.sleep()
