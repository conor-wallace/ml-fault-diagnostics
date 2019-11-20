import rospy
import std_msgs
from network_faults.msg import Velocity, ROSpid, Coordinate, Path
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
from pid import PID
from bicycle import Bicycle

time_series = []
x_error = []
y_error = []
t_error = []
d_error = []
desired_path = []
astar_path = []
k = []
first_read = 1
path_msg = None
path = []
stop = 0

def getGainCallback(msg):
    global k
    if len(msg.gain) != 0:
        k = msg.gain

def getPathCallback(msg):
    global first_read, path_msg
    if first_read:
        path_msg = msg.path
        first_read = 0

rospy.init_node('calc_pid', anonymous=True)
rospy.Subscriber("/gain", ROSpid, getGainCallback)
rospy.Subscriber("/path", Path, getPathCallback)
vel_pub = rospy.Publisher('/pid', Velocity, queue_size=1, tcp_nodelay=True)
lilbot_vel = Velocity()
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
i = 46
j = 10
rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    try:
        trans = tfBuffer.lookup_transform('odom', 'lilbot_SIMULATION/Base Frame', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rate.sleep()
        print "Error", e
        continue

    if not first_read:
        if not stop:
            print("first read")
            for point in path_msg:
                if -1000.0 not in point.coordinate:
                    path.append(list(point.coordinate))
                else:
                    path = np.array(path)
                    print(path)
                    print("STOP")
                    stop = 1

    if len(k) != 0 and stop == 1:
        pid = PID(k)
        bicycle = Bicycle(path)
        bicycle.pid = pid

        stamp_time = trans.header.stamp.to_sec()
        bicycle.x = round(trans.transform.translation.x, 3)
        bicycle.y = round(trans.transform.translation.y, 3)

        quaternion_trans = (
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w)
        euler_trans = euler_from_quaternion(quaternion_trans)
        bicycle.theta = round(euler_trans[2], 3)
        if i >= 0:
            bicycle.desired_x = bicycle.path[i, 0]
            bicycle.desired_y = bicycle.path[i, 1]
            delta_x = bicycle.desired_x - bicycle.x
            delta_y = bicycle.desired_y - bicycle.y
            print("delta x: %s" % delta_x)
            print("delta y: %s" % delta_y)
        else:
            astar_path = np.asarray(astar_path)
            plt.plot(astar_path[:, 0], astar_path[:, 1])
            plt.xlabel('x')
            plt.ylabel('y')
            plt.title('UGV Path')
            plt.show()

            time = np.arange(len(t_error))
            x_error = np.array(x_error)
            plt.plot(time[:], x_error[:, 0])
            plt.xlabel('t')
            plt.ylabel('x')
            plt.title('UGV X Trajectory')
            plt.show()

            y_error = np.array(y_error)
            plt.plot(time[:], y_error[:, 0])
            plt.xlabel('t')
            plt.ylabel('y')
            plt.title('UGV Y Trajectory')
            plt.show()

            time = np.arange(len(t_error))
            t_error = np.asarray(t_error)
            plt.plot(time[:], t_error[:, 0])
            plt.xlabel('t')
            plt.ylabel('theta')
            plt.title('UGV Theta Error')
            plt.show()

            d_error = np.asarray(d_error)
            plt.plot(time[:], d_error[:, 0])
            plt.xlabel('t')
            plt.ylabel('yl2 norm')
            plt.title('UGV Distance Error')
            plt.show()
            sys.exit(1)
        if j > 0:
            delta_x = bicycle.desired_x - bicycle.x
            delta_y = bicycle.desired_y - bicycle.y
            bicycle.desired_theta = math.atan2(delta_y, delta_x)
            delta_theta = bicycle.desired_theta - bicycle.theta

            distance = math.sqrt(delta_x**2 + delta_y**2)
            x_error.append([delta_x])
            y_error.append([delta_y])
            t_error.append([delta_theta])
            d_error.append([distance])
            desired_path.append([bicycle.desired_x, bicycle.desired_y, bicycle.desired_theta])
            astar_path.append([bicycle.x, bicycle.y, bicycle.theta])
            bicycle.pid.calculatePID(distance, delta_theta)
            bicycle.dynamics(bicycle.pid.v, bicycle.pid.g)
            j = j - 1
        else:
            i = i - 1
            j = bicycle.iter

        lilbot_vel.lin_vel = bicycle.pid.v
        lilbot_vel.ang_vel = bicycle.pid.g
        lilbot_vel.header.stamp = rospy.Time.now()
    vel_pub.publish(lilbot_vel)
    rate.sleep()
