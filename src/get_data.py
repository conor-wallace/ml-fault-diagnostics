#!/usr/bin/env python
import rospy

import sys
import csv
import geometry_msgs
import std_msgs
import tf2_ros
import tf_conversions
import numpy as np
from matplotlib import pyplot as plt
import message_filters
from network_faults.msg import Network, Velocity, IMU
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from message_filters import TimeSynchronizer, Subscriber

data = []
path = []
first_read = 1
first_time = 0.0
samples = 450
features = 7

rospy.init_node('GetData', anonymous=True)
rate = rospy.Rate(100.0)
publisher = 'lilbot_SIMULATION'
# publisher = 'lilbot_6ADE87'
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
stamp_iter = 0

while not rospy.is_shutdown():
    try:
        # trans = tfBuffer.lookup_transform(publisher+'/World Frame', publisher+'/Base Frame', rospy.Time())
        trans = tfBuffer.lookup_transform("odom", publisher+"/Base Frame", rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rate.sleep()
        print "Error", e
        continue

    if first_read:
        first_time = trans.header.stamp.to_sec()
        first_read = 0

    stamp_time = trans.header.stamp.to_sec() - first_time
    stamp_iter += 1
    dx_trans = round(trans.transform.translation.x, 3)
    dy_trans = round(trans.transform.translation.y, 3)
    print("x: %s y: %s" % (dx_trans, dy_trans))

    quaternion_trans = (
        trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w)
    euler_trans = euler_from_quaternion(quaternion_trans)
    roll_trans = euler_trans[0]
    pitch_trans = euler_trans[1]
    yaw_trans = euler_trans[2]

    data.append([stamp_time, stamp_iter, dx_trans, dy_trans, roll_trans, pitch_trans, yaw_trans])
    path.append([dx_trans, dy_trans])

    if len(data) == samples:
        print("converting data to csv")
        data = np.array(data)
        data = np.reshape(data, [samples, features])
        path = np.array(path)
        path = np.reshape(path, [samples, 2])

        # f=open("../data/lilbot_data.csv",'a')
        # np.savetxt(f, data, delimiter

        plt.scatter(path[:, 0], path[:, 1], color='blue')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.ylim(-0.1,0.6)
        plt.title('Healthy Path Data')
        plt.show()

        sys.exit(1)

    rate.sleep()
