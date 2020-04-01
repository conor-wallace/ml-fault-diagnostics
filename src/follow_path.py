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
        # f = open('/home/conor/catkin_ws/src/network_faults/data/path.csv', 'w')
        # np.savetxt(f, path, delimiter=",")
        trials = 100

        ideal_condition = 0
        ideal_bicycle = Bicycle(path)
        ideal_bicycle.createPath()

        healthy_fault_condition = 1
        healthy_fault_bicycle = Bicycle(path)
        healthy_fault_bicycle.createPath()
        healthy_fault_bicycle.readNoiseFunction()
        healthy_fault_bicycle.setNoiseFunction(healthy_fault_condition)
        healthy_fault_bicycle.driveOpenLoop(healthy_fault_condition, 'green')

        left_fault_condition = 2
        left_fault_bicycle = Bicycle(path)
        left_fault_bicycle.createPath()
        left_fault_bicycle.readNoiseFunction()
        left_fault_bicycle.setNoiseFunction(left_fault_condition)
        left_fault_bicycle.driveOpenLoop(left_fault_condition, 'red')

        right_fault_condition = 3
        right_fault_bicycle = Bicycle(path)
        right_fault_bicycle.createPath()
        right_fault_bicycle.readNoiseFunction()
        right_fault_bicycle.setNoiseFunction(right_fault_condition)
        right_fault_bicycle.driveOpenLoop(right_fault_condition, 'blue')

        for _ in range(trials):
            healthy_fault_bicycle.readNoiseFunction()
            healthy_fault_bicycle.setNoiseFunction(healthy_fault_condition)
            left_fault_bicycle.readNoiseFunction()
            left_fault_bicycle.setNoiseFunction(left_fault_condition)
            right_fault_bicycle.readNoiseFunction()
            right_fault_bicycle.setNoiseFunction(right_fault_condition)

            ideal_bicycle.driveAlongPath(0, ideal_bicycle.pid, None, 0, ideal_condition)
            healthy_fault_bicycle.driveAlongPath(0, healthy_fault_bicycle.pid, None, 0, healthy_fault_condition)
            left_fault_bicycle.driveAlongPath(0, left_fault_bicycle.pid, None, 0, left_fault_condition)
            right_fault_bicycle.driveAlongPath(0, right_fault_bicycle.pid, None, 0, right_fault_condition)

            ideal_path = ideal_bicycle.path_data
            healthy_path = healthy_fault_bicycle.path_data
            left_path = left_fault_bicycle.path_data
            right_path = right_fault_bicycle.path_data

            ideal_path = np.reshape(ideal_path, (920, 13))
            healthy_path = np.reshape(healthy_path, (920, 13))
            left_path = np.reshape(left_path, (920, 13))
            right_path = np.reshape(right_path, (920, 13))

            print(healthy_path.shape)
            print(left_path.shape)
            print(right_path.shape)

            healthy_path[:, 0:3] = ideal_path[:, 0:3] - healthy_path[:, 0:3]
            left_path[:, 0:3] = ideal_path[:, 0:3] - left_path[:, 0:3]
            right_path[:, 0:3] = ideal_path[:, 0:3] - right_path[:, 0:3]

            path = healthy_path
            path = np.concatenate((path, left_path), axis=0)
            path = np.concatenate((path, right_path), axis=0)

            print(path.shape)
            f = open('/home/conor/catkin_ws/src/network_faults/data/path.csv', 'a')
            np.savetxt(f, path, delimiter=",")

        # generations = 100
        #
        # healthy_fault_condition = 1
        # healthy_fault_bicycle = Bicycle(path)
        # healthy_fault_bicycle.createPath()
        # healthy_fault_bicycle.readNoiseFunction()
        # healthy_fault_bicycle.setNoiseFunction(healthy_fault_condition)
        # healthy_fault_bicycle.driveOpenLoop(healthy_fault_condition, 'green')
        #
        # for i in range(1):
        #     healthy_residual_path_data = []
        #     healthy_fault_bicycle.driveAlongPath(0, healthy_fault_bicycle.pid, None, 1, healthy_fault_condition)
        #     healthy_residual_path_data = healthy_fault_bicycle.astar_path.copy()
        #     healthy_residual_path_data = healthy_fault_bicycle.desired_path[20:, :] - healthy_residual_path_data
        #
        #     healthy_ga = GA(100, generations, healthy_fault_bicycle, healthy_fault_condition)
        #     healthy_ga.setup()
        #     healthy_optimal_k = healthy_ga.evolve()
        #     healthy_optimal_k = list(healthy_optimal_k)
        #     pid_msg.gain = healthy_optimal_k
        #     print(healthy_optimal_k)
        #     healthy_residual_path_data_opt = healthy_fault_bicycle.astar_path.copy()
        #     healthy_residual_path_data_opt = healthy_fault_bicycle.desired_path[20:, :] - healthy_residual_path_data_opt
        #     plt.plot(np.arange(healthy_residual_path_data.shape[0]), healthy_residual_path_data[:, 0], color='red', label='Healthy theta response')
        #     plt.plot(np.arange(healthy_residual_path_data.shape[0]), healthy_residual_path_data_opt[:, 0], color='blue', label='Healthy theta response optimized')
        #     plt.xlabel('time')
        #     plt.ylabel('residual (rad)')
        #     plt.legend()
        #     plt.show()
        #
        # left_fault_condition = 2
        # left_fault_bicycle = Bicycle(path)
        # left_fault_bicycle.createPath()
        # left_fault_bicycle.readNoiseFunction()
        # left_fault_bicycle.setNoiseFunction(left_fault_condition)
        # left_fault_bicycle.driveOpenLoop(left_fault_condition, 'red')
        #
        # for j in range(1):
        #     left_residual_path_data = []
        #     left_fault_bicycle.driveAlongPath(0, left_fault_bicycle.pid, None, 1, left_fault_condition)
        #     left_residual_path_data = left_fault_bicycle.path_data.copy()
        #     left_residual_path_data[:, 0:3] = left_residual_path_data[:, 0:3] - ideal_bicycle.path_data[:, 0:3]
        #     left_ga = GA(100, generations, left_fault_bicycle, left_fault_condition)
        #     left_ga.setup()
        #     left_optimal_k = left_ga.evolve()
        #     left_optimal_k = list(left_optimal_k)
        #     pid_msg.gain = left_optimal_k
        #     print(left_optimal_k)
        #     left_residual_path_data_opt = left_fault_bicycle.path_data.copy()
        #     left_residual_path_data_opt[:, 0:3] = left_residual_path_data_opt[:, 0:3] - ideal_bicycle.path_data[:, 0:3]
        #     plt.plot(left_residual_path_data[:, 5], left_residual_path_data[:, 2], color='red', label='Left fault theta response')
        #     plt.plot(left_residual_path_data_opt[:, 5], left_residual_path_data_opt[:, 2], color='blue', label='Left fault theta response optimized')
        #     plt.xlabel('time')
        #     plt.ylabel('residual (rad)')
        #     plt.legend()
        #     plt.show()
        #
        # right_fault_condition = 3
        # right_fault_bicycle = Bicycle(path)
        # right_fault_bicycle.createPath()
        # right_fault_bicycle.readNoiseFunction()
        # right_fault_bicycle.setNoiseFunction(right_fault_condition)
        # right_fault_bicycle.driveOpenLoop(right_fault_condition, 'blue')
        #
        # for k in range(1):
        #     right_residual_path_data = []
        #     right_fault_bicycle.driveAlongPath(0, right_fault_bicycle.pid, None, 1, right_fault_condition)
        #     right_residual_path_data = right_fault_bicycle.path_data.copy()
        #     right_residual_path_data[:, 0:3] = right_residual_path_data[:, 0:3] - ideal_bicycle.path_data[:, 0:3]
        #     right_ga = GA(100, generations, right_fault_bicycle, right_fault_condition)
        #     right_ga.setup()
        #     right_optimal_k = right_ga.evolve()
        #     right_optimal_k = list(right_optimal_k)
        #     pid_msg.gain = right_optimal_k
        #     print(right_optimal_k)
        #     right_residual_path_data_opt = right_fault_bicycle.path_data.copy()
        #     right_residual_path_data_opt[:, 0:3] = right_residual_path_data_opt[:, 0:3] - ideal_bicycle.path_data[:, 0:3]
        #     plt.plot(right_residual_path_data[:, 5], right_residual_path_data[:, 2], color='red', label='Right fault theta response')
        #     plt.plot(right_residual_path_data_opt[:, 5], right_residual_path_data_opt[:, 2], color='blue', label='Right fault theta response optimized')
        #     plt.xlabel('time')
        #     plt.ylabel('residual (rad)')
        #     plt.legend()
        #     plt.show()

        #
        # dict_file = dict(Matrix=np.array(optimal_k))
        #
        # with open('/home/conor/catkin_ws/src/network_faults/data/pid_file.yaml', 'w') as file:
        #     documents = yaml.dump(dict_file, file, default_flow_style=False)

        sys.exit(1)
    pid_pub.publish(pid_msg)
    rate.sleep()
