import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import rospy
from network_faults.msg import Coordinate, Path, ROSpid
from pid import PID
from bicycle import Bicycle
from ga_pid import GA

first_read = 1
path_msg = None
path = []
stop_criterion = [-1000.0, -1000.0]
stop = 0
test_theta = 0.0
test_x = 0.0
test_y = 0.0
target = 0

def getPathCallback(msg):
    global first_read, path_msg
    if first_read:
        path_msg = msg.path
        first_read = 0

rospy.init_node('follow_path')
rospy.Subscriber("/path", Path, getPathCallback)
pid_pub = rospy.Publisher('/gain', ROSpid, queue_size=1, tcp_nodelay=True)
pid_msg = ROSpid()

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
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

    if stop and not target:
        ga = GA(100, 0.4, 100, Bicycle(path))
        ga.setup()
        optimal_k = ga.evolve()
        pid_msg.gain = list(optimal_k)

        target = 1
    pid_pub.publish(pid_msg)
    rate.sleep()
