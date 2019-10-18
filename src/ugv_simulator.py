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

stop = 0
V_PID = 0
G_PID = 0
V_Proportional = 0
V_Derivative = 0
V_Integral = 0
V_Derivator = 0
V_Integrator = 0
G_Proportional = 0
G_Derivative = 0
G_Integral = 0
G_Derivator = 0
G_Integrator = 0
Kv_pt = 2
Kv_it = 0
Kv_dt = 0
Kh_pt = 0.1 #33
Kh_it = 0.0
Kh_dt = 0.0 #10
v = 0.0
gamma = 0.0
time_series = []
test_path = []
first_time = 0
stamp_time = 0
last_time = 0
first_read = 0
x = 0
stop = 0

test_theta = 0
test_x = 0.0
test_y = 0.0

target_theta = 0.0
dx_target = 0.0
dy_target = 0.0
v_target = 0.1
gamma_target = 30

def getPIDCallback(data):
    global v, gamma, max_vel_w, max_vel_x, first_read
    print(data.lin_vel, data.ang_vel)
    if data.lin_vel == 1000:
        stop = 1
    if (first_read == 0):
        first_read = 1
        v = data.lin_vel
        gamma = data.ang_vel
    else:
        v = data.lin_vel
        gamma = data.ang_vel

rospy.init_node('ugv_simulator')
rospy.Subscriber("/pid", Velocity, getPIDCallback)
pose_pub = rospy.Publisher("lilbot_SIMULATION/pose", PoseStamped, queue_size=1, tcp_nodelay=True)

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    current_time = rospy.get_time()
    # perform the broadcasting
    noise_level = np.random.normal(0.0, 0.01)*0.01*0.5
    print("noise: %s" % noise_level)
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom";#"base_link"
    t.child_frame_id = "lilbot_SIMULATION/Base Frame"
    t.transform.translation.x = test_x+noise_level
    t.transform.translation.y = test_y+noise_level
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, test_theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    rospy.Time.now()
    br.sendTransform(t)

    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id =  "odom"
    t.child_frame_id = "lilbot_SIMULATION/desired_pose"
    t.transform.translation.x = dx_target
    t.transform.translation.y = dy_target
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, target_theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    rospy.Time.now()
    br.sendTransform(t)

    delta_x = dx_target - test_x
    delta_y = dy_target - test_y

    desired_heading = (math.degrees(math.atan2((delta_y), (delta_x)) % (2 * math.pi)))
    # print("desired heading: %s" % desired_heading)
    current_heading = (math.degrees(test_theta % (2 * math.pi)))
    # print("current heading: %s" % current_heading)

    #if(desired_heading < 180):
     #   desired_heading += 360
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

    L = 0.19
    theta_dot = ((v/L)*(math.tan(gamma)))
    x_dot = v * math.cos(test_theta)
    y_dot = v * math.sin(test_theta)
    test_theta = test_theta + theta_dot*delta_t
    test_x = test_x + x_dot*delta_t
    test_y = test_y + y_dot*delta_t

    target_theta_dot = ((v_target/L)*(math.tan(gamma_target)))
    target_x_dot = v_target * math.cos(target_theta)
    target_y_dot = v_target * math.sin(target_theta)
    target_theta = target_theta + target_theta_dot*delta_t
    dx_target = dx_target + target_x_dot*delta_t
    dy_target = dy_target + target_y_dot*delta_t

    if stop == 1:
        sys.exit(1)
    rate.sleep()
