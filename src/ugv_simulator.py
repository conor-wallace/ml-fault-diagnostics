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

stop = 0
max_rad = 38.0
max_vel = 1.0
v_scale = max_vel / 255.0
g_scale = max_rad / 255.0
v = 150 * v_scale
gamma = 0.0
time_series = []
test_path = []
first_time = 0
stamp_time = 0
last_time = 0
first_read = 0
x = 0
stop = 0
count = 0
L = 0.19
prev_noise = 0.0

test_theta = 0
test_x = 0.0
test_y = 0.0

# def getPIDCallback(data):
#     global v, gamma, max_vel_w, max_vel_x, first_read
#     print(data.lin_vel, data.ang_vel)
#     if data.lin_vel == 1000:
#         stop = 1
#     if (first_read == 0):
#         first_read = 1
#         v = data.lin_vel
#         gamma = data.ang_vel
#     else:
#         v = data.lin_vel
#         gamma = data.ang_vel

rospy.init_node('ugv_simulator')
# rospy.Subscriber("/pid", Velocity, getPIDCallback)
pose_pub = rospy.Publisher("lilbot_SIMULATION/pose", PoseStamped, queue_size=1, tcp_nodelay=True)

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    current_time = rospy.get_time()
    if first_read == 0:
        noise_level = 0.0
        fault_level = 0.0
        x = rospy.Time.now().to_sec()*0.5
        delta_x = rospy.Time.now().to_sec()*0.5 - x
        dx_target = 0.0
        dy_target = 0.0
        dx_trans = 0
        dy_trans = 0
        yaw_trans = 0
    else:
        delta_x = rospy.Time.now().to_sec()*0.5 - x
        noise_level = np.random.normal(0.01, 0.001)
        print("time: %s" % delta_x)
        fault_level = 0.0*(np.exp(0.005*count)-1)
        print("fault level: %s" % fault_level)
        count += 1
        dx_target = -1 * (math.sin(delta_x+np.pi))
        dy_target = -1 * (math.cos(delta_x+np.pi)+1)
        dx_trans = test_x
        dy_trans = test_y
        yaw_trans = test_theta
    # perform the broadcasting
    # print("noise: %s" % noise_level)
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"#"base_link"
    t.child_frame_id = "lilbot_SIMULATION/Base Frame"
    t.transform.translation.x = test_x
    t.transform.translation.y = test_y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, test_theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    rospy.Time.now()
    br.sendTransform(t)

    test_path.append([dx_trans, dy_trans])
    delta_x = dx_target - test_x
    delta_y = dy_target - test_y

    desired_heading = (math.degrees(math.atan2((delta_y), (delta_x)) % (2 * math.pi)))
    current_heading = (math.degrees(test_theta % (2 * math.pi)))

    if desired_heading >= 270.0 and desired_heading <= 360.0:
        if current_heading >= 0.0 and current_heading <= 90.0:
            current_heading += 360
    delta_theta = desired_heading - current_heading

    curr_time = rospy.Time.now().to_sec()
    if not first_time:
        first_time = curr_time
    time_series.append((curr_time - first_time)*1000)
    delta_t = (curr_time - last_time)
    delta_t = (delta_t if delta_t != 0 else 0.1)
    last_time = curr_time

    #dynamics
    theta_dot = ((v/L)*(math.tan(gamma)))
    x_dot = v * math.cos(test_theta)
    y_dot = v * math.sin(test_theta)

    a = -0.494
    b = -0.487
    c = 0.488

    noise = a * np.exp(b * test_x) + c
    delta_t = 0.1 #10 ms
    test_theta = test_theta + np.clip(theta_dot, -1*math.radians(max_rad), math.radians(max_rad))*delta_t
    test_x = test_x + np.clip(x_dot, -1*max_vel, max_vel)*delta_t
    test_y = test_y + np.clip(y_dot, -1*max_vel, max_vel)*delta_t+(noise-prev_noise)
    prev_noise = noise

    if len(test_path) == 45:
        print("stop")
        sys.exit(1)
    rate.sleep()
