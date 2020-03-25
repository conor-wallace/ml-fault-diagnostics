import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import rospy
from network_faults.msg import Coordinate, Path, ROSpid
from pid import PID
from bicycle import Bicycle
from ga_pid import GA
import yaml

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
        # print(path)
        # f = open('/home/ace/catkin_ws/src/network_faults/data/path.csv', 'w')
        # np.savetxt(f, path, delimiter=",")
        ideal_condition = 0
        ideal_bicycle = Bicycle(path)
        ideal_bicycle.createPath()
        ideal_bicycle.driveAlongPath(0, ideal_bicycle.pid, None, 1, ideal_condition)

        healthy_fault_condition = 1
        healthy_fault_bicycle = Bicycle(path)
        healthy_fault_bicycle.createPath()
        healthy_fault_bicycle.readNoiseFunction()
        healthy_fault_bicycle.setNoiseFunction(healthy_fault_condition)
        healthy_fault_bicycle.driveOpenLoop(healthy_fault_condition, 'green')

        for i in range(1):
            healthy_residual_path_data = []
            healthy_fault_bicycle.driveAlongPath(0, healthy_fault_bicycle.pid, None, 1, healthy_fault_condition)
            # healthy_residual_path_data = healthy_fault_bicycle.path_data.copy()
            # healthy_residual_path_data[:, 0:3] = healthy_residual_path_data[:, 0:3] - ideal_bicycle.path_data[:, 0:3]
            # f = open('/home/ace/catkin_ws/src/network_faults/data/path_data.csv', 'a')
            # np.savetxt(f, healthy_residual_path_data, delimiter=",")
            healthy_ga = GA(100, 500, healthy_fault_bicycle, healthy_fault_condition)
            healthy_ga.setup()
            healthy_optimal_k = healthy_ga.evolve()
            healthy_optimal_k = list(healthy_optimal_k)
            pid_msg.gain = healthy_optimal_k
            print(healthy_optimal_k)

        left_fault_condition = 2
        left_fault_bicycle = Bicycle(path)
        left_fault_bicycle.createPath()
        left_fault_bicycle.readNoiseFunction()
        left_fault_bicycle.setNoiseFunction(left_fault_condition)
        left_fault_bicycle.driveOpenLoop(left_fault_condition, 'red')

        for j in range(1):
            left_residual_path_data = []
            left_fault_bicycle.driveAlongPath(0, left_fault_bicycle.pid, None, 1, left_fault_condition)
        #     left_residual_path_data = left_fault_bicycle.path_data.copy()
            # left_residual_path_data[:, 0:3] = left_residual_path_data[:, 0:3] - ideal_bicycle.path_data[:, 0:3]
            # f = open('/home/ace/catkin_ws/src/network_faults/data/path_data.csv', 'a')
            # np.savetxt(f, left_residual_path_data, delimiter=",")
            left_ga = GA(100, 500, left_fault_bicycle, left_fault_condition)
            left_ga.setup()
            left_optimal_k = left_ga.evolve()
            left_optimal_k = list(left_optimal_k)
            pid_msg.gain = left_optimal_k
            print(left_optimal_k)

        right_fault_condition = 3
        right_fault_bicycle = Bicycle(path)
        right_fault_bicycle.createPath()
        right_fault_bicycle.readNoiseFunction()
        right_fault_bicycle.setNoiseFunction(right_fault_condition)
        right_fault_bicycle.driveOpenLoop(right_fault_condition, 'blue')

        for k in range(1):
            right_residual_path_data = []
            right_fault_bicycle.driveAlongPath(0, right_fault_bicycle.pid, None, 1, right_fault_condition)
        #     right_residual_path_data = right_fault_bicycle.path_data.copy()
            # right_residual_path_data[:, 0:3] = right_residual_path_data[:, 0:3] - ideal_bicycle.path_data[:, 0:3]
            # f = open('/home/ace/catkin_ws/src/network_faults/data/path_data.csv', 'a')
            # np.savetxt(f, right_residual_path_data, delimiter=",")
            right_ga = GA(100, 500, right_fault_bicycle, right_fault_condition)
            right_ga.setup()
            right_optimal_k = right_ga.evolve()
            right_optimal_k = list(right_optimal_k)
            pid_msg.gain = right_optimal_k
            print(right_optimal_k)

        #
        # dict_file = dict(Matrix=np.array(optimal_k))
        #
        # with open('/home/conor/catkin_ws/src/network_faults/data/pid_file.yaml', 'w') as file:
        #     documents = yaml.dump(dict_file, file, default_flow_style=False)

        sys.exit(1)
    pid_pub.publish(pid_msg)
    rate.sleep()
