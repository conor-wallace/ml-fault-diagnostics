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
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
from pid import PID
from bicycle import Bicycle


def driveUGV(model):
    model.pid.k = [0.01, 0.0, 0.0, 1.0, 0.0, 0.0]

    rospy.init_node('ugv_simulator')
    # rospy.Subscriber("/pid", Velocity, getPIDCallback)
    pose_pub = rospy.Publisher("/model_odom", PoseStamped, queue_size=1, tcp_nodelay=True)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)

    test_path = []
    last_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "/model_base_link"#"base_link"
        t.child_frame_id = "/model_base_frame"
        t.transform.translation.x = model.x
        t.transform.translation.y = model.y
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, model.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        rospy.Time.now()
        br.sendTransform(t)

        test_path.append([model.x, model.y])
        current_time = rospy.Time.now().to_sec()
        dt = current_time - last_time
        print(dt)
        last_time = current_time

        # PID stuff goes here
        model.calculateError(model.path)
        model.pid.calculatePID(model.distance_error, model.heading_error, dt)
        velocity = model.pid.velocity
        steering = model.pid.steering

        # Fault classiification: 0 = healthy noise, 1 = left noise, 2 = right noise, 3 = no noise
        fault = 3

        # Actuation
        model.dynamics(velocity, steering, 3, dt)
        distance = np.linalg.norm(np.array([model.x, model.y] - model.path))
        print(np.array([model.x, model.y]))
        print(model.path)

        if (distance <= 0.5):
            print("stop")
            return test_path
        rate.sleep()

if __name__ == '__main__':
    target = np.array([10.0, 10.0])
    bicycle = Bicycle(target)
    achieved_path = driveUGV(bicycle)

    achieved_path = np.array(achieved_path)
    plt.plot(achieved_path[:, 0], achieved_path[:, 1])
    plt.show()
