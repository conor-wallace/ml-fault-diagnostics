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
global xv, wv, last_x, last_last_x, xv1, xv2, g_enabled, publishers, DEBUG_ENABLED, last_time, g_testing_enabled

last_x = dict()
last_last_x = dict()
publishers = dict()
last_time = 0
xv = wv = xv1 = xv2 = 0
last_x = dict()
last_last_x = dict()
publishers = dict()
velocities = dict()
last_yaw_diff2 = dict()
last_d = dict()
last_time = dict()
last_delta_t = dict()
last_check_time = 0
xv = wv = xv1 = xv2 = 0
max_vel_x = 1
max_vel_w = 30
segments = 64
increment = 48
total_increments = 1
x_center = 0
y_center = 1
radius = 1
L = 0.19
v = 0.0
gamma = 0.0
path_target = []
path = []
#255 = max turn rate for motors, 30 = max turn radius for drive shaft in degrees
turn_rate_factor = 250/30
#255 = max forward velocity of motors, 5 = max forward velocity in m/s
velocity_factor = 250/4.19
dx = 0
dy = 0
th0 = round(((increment-1)*2*math.pi)/segments, 3)
th = round(((increment)*2*math.pi)/segments, 3)
#desired_x = 0.7
#desired_y = 0.3
desired_x = round((x_center+radius*math.cos(th)), 3)
desired_y = round((y_center+radius*math.sin(th)), 3)
first_point_x = desired_x
first_point_y = desired_y
desired_theta = (th0)
desired_z = 0
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
time_series = []
x_pos = []
y_pos = []
first_time = 0
stamp_time = 0
last_time = 0
error = []
test_path = []
x = 0

test_theta = 0
test_x = 0.0
test_y = 0.0
desired_x = 2.5
desired_y = 1
desired_theta= 0

pose_pub = rospy.Publisher("lilbot_SIMULATION/pose", PoseStamped, queue_size=1, tcp_nodelay=True)

def enableCallback(data):
    global g_enabled
    g_enabled = data.data
    rospy.loginfo("System Control Enable = %s", g_enabled)

def getPIDCallback(data):
    global v, gamma, max_vel_w, max_vel_x
    v = data.lin_vel
    gamma = data.ang_vel
    v = np.clip(v, -max_vel_x, max_vel_x)
    gamma = np.clip(gamma, -max_vel_w, max_vel_w)

def updateTopicList():
    global publishers, last_time
    topic_list = rospy.get_published_topics()
    i = 0
    last_time = rospy.get_time()
    for topic in topic_list:
       # print "topic is ", topic[0]
       if "pose" in topic[0]	 and "lilbot" in topic[0]:
          ns = topic[0].split("/")[1]

          if DEBUG_ENABLED:
             # print "Topic: [%s] is a lilbot topic" % topic[0]
             print "ns is ", ns

          if ns not in publishers.keys():
             publishers[ns] = rospy.Publisher(ns + "/cmd", Int16MultiArray, queue_size=1, tcp_nodelay=True)
             # print "Found a new lilbot pose topic, adding to control list"
          last_x[ns] = 0
          last_last_x[ns] = 0
          i = i + 1

if __name__ == '__main__':

    rospy.init_node('lilbot_controller_tf2_listener')
    rospy.Subscriber("enable_motors", Int16, enableCallback)
    rospy.Subscriber("pid", Velocity, getPIDCallback)
    desired_pose = PoseStamped()
    #desired_pose.header.frame_id = "/lilbot_3BA615/World Frame"

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    updateTopicList()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        if not first_time:
            x = rospy.Time.now().to_sec()*0.5
        delta_x = rospy.Time.now().to_sec()*0.5 - x
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id =  "lilbot_3BA615/World Frame"
        t.child_frame_id = "lilbot_3BA615/desired_pose"
        t.transform.translation.x = -1 * (math.sin(delta_x+np.pi))#+np.random.rand()*noise_level
        t.transform.translation.y = -1 * (math.cos(delta_x+np.pi)+1)#desired_y+np.random.rand()*noise_level
        t.transform.translation.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, desired_theta+np.random.rand()*0.01)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        rospy.Time.now()
        br.sendTransform(t)

        current_time = rospy.get_time()
         try:
            generateTestData("lilbot_SIMULATION")
         except:
            pass

        for publisher in publishers.keys():
            if not first_time:
                x = rospy.Time.now().to_sec()*0.5
            delta_x = rospy.Time.now().to_sec()*0.5 - x
            dx_target = -1 * (math.sin(delta_x+np.pi))
            dy_target = -1 * (math.cos(delta_x+np.pi)+1)
            dx_trans = test_x
            dy_trans = test_y
            yaw_trans = test_theta
            # perform the broadcasting
            br = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "odom";#"base_link"
            t.child_frame_id = robot_ns + "/Base Frame"
            t.transform.translation.x = test_x+np.random.rand()*noise_level
            t.transform.translation.y = test_y+np.random.rand()*noise_level
            t.transform.translation.z = 0+np.random.rand()*noise_level
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, test_theta+np.random.rand()*0.01)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            rospy.Time.now()
            br.sendTransform(t)

            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id =  "odom"
            t.child_frame_id = robot_ns + "/desired_pose"
            t.transform.translation.x = dx_target#+np.random.rand()*noise_level
            t.transform.translation.y = dy_target#desired_y+np.random.rand()*noise_level
            t.transform.translation.z = 0+np.random.rand()*noise_level
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, desired_theta+np.random.rand()*0.01)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            rospy.Time.now()
            br.sendTransform(t)

            pose = PoseStamped()
            pose.header.stamp = t.header.stamp #rospy.Time.now()
            pose.header.frame_id = robot_ns + "/Base Frame"
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0


            quaternion = tf_conversions.transformations.quaternion_from_euler(0, 0, test_theta)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            pose_pub.publish(pose)

            v_error = round(math.sqrt(math.pow((dx_target - dx_trans), 2) + math.pow((dy_target - dy_trans), 2)), 5)
            desired_heading = (math.degrees(math.atan2((dy_target - dy_trans), (dx_target - dx_trans)) % (2 * math.pi)))
            #print("desired heading: %s" % desired_heading)
            current_heading = (math.degrees(yaw_trans % (2 * math.pi)))
            #print("current heading: %s" % current_heading)

            if desired_heading >= 270.0 and desired_heading <= 360.0:
                if current_heading >= 0.0 and current_heading <= 90.0:
                    current_heading += 360
            gamma_error = desired_heading - current_heading
            error.append(gamma_error)
            #print("v error: %s" % v_error)
            #print("gamma error: %s" % gamma_error)
            test_path.append([dx_trans, dy_trans])
            #print(len(test_path))

            curr_time = rospy.Time.now().to_sec()
            if not first_time:
                first_time = curr_time
            #print(curr_time - first_time)
            time_series.append((curr_time - first_time)*1000)
            delta_t = (curr_time - last_time)
            delta_t = (delta_t if delta_t != 0 else 0.1)
            print delta_t
            last_time = curr_time

            print v, gamma

            L = 0.2
            lr = 0.1
            beta = math.atan((lr * math.tan(gamma) / L))
            x_dot = v * math.cos(test_theta + beta)
            y_dot = v * math.sin(test_theta + beta)
            theta_dot = v * math.cos(beta) * math.tan(beta) / L
            delta_dot = gamma
            test_theta = test_theta + theta_dot*delta_t
            test_x = test_x + x_dot*delta_t
            test_y = test_y + y_dot*delta_t
            d = math.sqrt((test_x-desired_x)**2+(test_y-desired_y)**2)
            print test_x, test_y
            print publisher

            xv1 = xv2 = xv = wv = 0
            msg = Int16MultiArray()
            if len(test_path) > 200:
                msg.data = [0,0,0,0]
                publishers[publisher].publish(msg)
                # print "publishing"
                test_path = np.asarray(test_path)

                plt.plot(test_path[:, 0], test_path[:, 1], label='UGV Position')
                plt.xlabel('x')
                plt.ylabel('y')
                plt.title('UGV Path Error')
                plt.legend()
                plt.show()

                sys.exit(1)
            else:
                # Negation is because wires are backwards... could fix this sometime.
                msg.data = [int(-xv1),int(-xv2),0,int(-wv)]
                publishers[publisher].publish(msg)
                # print "publishing"

        rate.sleep()
