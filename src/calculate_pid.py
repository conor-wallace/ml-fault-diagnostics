import rospy
import std_msgs
from network_faults.msg import Velocity
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
import numpy as np
import sys
import math
import tf2_ros
import tf_conversions
import geometry_msgs.msg

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
first_time = 0
stamp_time = 0
last_time = 0
max_vel_x = 5
max_vel_w = 30

def get_PID(v_error, gamma_error, delta_t):
    global Kv_pt, Kv_it, Kv_dt, Kh_pt, Kh_it, Kh_dt, V_Derivator, V_Integrator, G_Derivator, G_Integrator, max_vel_w, max_vel_x
    # PID for Distance error

    print v_error, gamma_error
    V_Proportional = v_error * Kv_pt

    V_Derivative = ((v_error - V_Derivator) * Kv_dt) / delta_t
    V_Derivator = v_error

    V_Integrator = v_error + V_Integrator
    V_Integrator = np.clip(V_Integrator, -max_vel_x, max_vel_x)
    V_Integral = V_Integrator * Kv_it

    V_PID = V_Proportional + V_Integral + V_Derivative

    # PID for Heading error
    G_Proportional = gamma_error * Kh_pt

    G_Derivative = ((gamma_error - G_Derivator) * Kh_dt) / delta_t
    G_Derivator = gamma_error

    G_Integrator = gamma_error + G_Integrator
    G_Integrator = np.clip(G_Integrator, -max_vel_w, max_vel_w)
    G_Integral = G_Integrator * Kh_it

    G_PID = G_Proportional + G_Integral + G_Derivative

    print("v P: %s" % V_Proportional)
    print("v I: %s" % V_Integral)
    print("v D: %s" % V_Derivative)
    print("v PID: %s" % V_PID)

    print("gamma P: %s" % G_Proportional)
    print("gamma I: %s" % G_Integral)
    print("gamma D: %s" % G_Derivative)
    print("gamma PID: %s" % G_PID)

    return V_PID, G_PID

rospy.init_node('calc_pid', anonymous=True)
vel_pub = rospy.Publisher('/pid', Velocity, queue_size=1, tcp_nodelay=True)
lilbot_vel = Velocity()
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    try:
        trans = tfBuffer.lookup_transform('odom', 'lilbot_SIMULATION/Base Frame', rospy.Time())
        trans_to_target = tfBuffer.lookup_transform('odom', "lilbot_SIMULATION/desired_pose", rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rate.sleep()
        print "Error", e
        continue

    dx_target = round(trans_to_target.transform.translation.x, 3)
    dy_target = round(trans_to_target.transform.translation.y, 3)
    print("x*: %s y*: %s" % (dx_target, dy_target))

    stamp_time = trans.header.stamp.to_sec()
    dx_trans = round(trans.transform.translation.x, 3)
    dy_trans = round(trans.transform.translation.y, 3)
    print("x: %s y: %s" % (dx_trans, dy_trans))

    quaternion_target = (
        trans_to_target.transform.rotation.x,
        trans_to_target.transform.rotation.y,
        trans_to_target.transform.rotation.z,
        trans_to_target.transform.rotation.w)
    euler_target = euler_from_quaternion(quaternion_target)
    roll_target = euler_target[0]
    pitch_target = euler_target[1]
    yaw_target = round(euler_target[2], 3)

    quaternion_trans = (
        trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w)
    euler_trans = euler_from_quaternion(quaternion_trans)
    roll_trans = euler_trans[0]
    pitch_trans = euler_trans[1]
    yaw_trans = round(euler_trans[2], 3)

    desired_heading = (math.degrees(math.atan2((dy_target - dy_trans), (dx_target - dx_trans)) % (2 * math.pi)))
    # print("desired heading: %s" % desired_heading)
    current_heading = (math.degrees(yaw_trans % (2 * math.pi)))
    # print("current heading: %s" % current_heading)

    #if(desired_heading < 180):
     #   desired_heading += 360
    if desired_heading >= 270.0 and desired_heading <= 360.0:
        if current_heading >= 0.0 and current_heading <= 90.0:
            current_heading += 360
    gamma_error = desired_heading - current_heading

    v_error = round(math.sqrt(math.pow((dx_target - dx_trans), 2) + math.pow((dy_target - dy_trans), 2)), 5)

    # print("v error: %s" % v_error)
    # print("gamma error: %s" % gamma_error)

    curr_time = trans.header.stamp.to_sec()
    if not first_time:
        first_time = curr_time

    # print(curr_time - first_time)
    delta_t = (stamp_time - last_time)
    delta_t = (delta_t if delta_t != 0 else 0.1)

    v, gamma = get_PID(v_error, gamma_error, delta_t)
    v = np.clip(v, -max_vel_x, max_vel_x)
    gamma = np.clip(gamma, -max_vel_w, max_vel_w)
    last_time = curr_time
    print("v gamma: %s %s" % (v, gamma))
    print("")

    lilbot_vel.lin_vel = int(v)
    lilbot_vel.ang_vel = int(gamma)
    lilbot_vel.header.stamp = rospy.Time.now()
    vel_pub.publish(lilbot_vel)
    rate.sleep()
