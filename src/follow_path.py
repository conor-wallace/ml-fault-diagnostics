import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import rospy
from network_faults.msg import Coordinate, Path
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
        k = [0.1, 0.0, 0.0, 1, 0.0, 0.0]
        # test_pid = PID(k)
        ga = GA(100, 0.35, 100, Bicycle(path))
        ga.setup()
        # ga.population[0] = test_pid
        ga.evolve()

        target = 1
    rate.sleep()
