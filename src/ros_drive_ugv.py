#!/usr/bin/env python

import rospy
import math
import sys
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose
from bicycle import Bicycle
from unity_controller.msg import AgentVelocity, AgentPose
import pandas as pd

ideal_input = None
faulty_input = None

def inputsCallback(msg):
    global ideal_input, faulty_input
    # print(msg)
    ideal_input = msg.agent_velocities[0]
    faulty_input = msg.agent_velocities[1]

def driveClosedLoop(ideal_ugv, faulty_ugv, ideal_condition, fault_condition):
    global ideal_input, faulty_input
    # initialize node
    rospy.init_node('fault_ugvs', anonymous = True)

    i = 44
    start_point = np.array([ideal_ugv.x, ideal_ugv.y])
    labels = ["ideal", "no fault", "left fault", "right fault"]
    rospy.Subscriber('agent_velocities', AgentVelocity, inputsCallback, queue_size=1, tcp_nodelay=True)
    agents_publisher = rospy.Publisher('agent_poses', AgentPose, queue_size=1, tcp_nodelay=True)
    rate = rospy.Rate(10) # 10hz
    msg = AgentPose()
    ideal_pose = Pose()
    faulty_pose = Pose()
    msg.agents = [ideal_pose, faulty_pose]
    time = rospy.Time.now().to_sec()
    iteration = 0
    start_time = 0.0
    start_index = 0
    first_measurement = ideal_ugv.x
    first_move = True
    current_index = 0
    previous_time = 0.0
    j = ideal_ugv.iter
    target = ideal_ugv.path[i, :]

    while not rospy.is_shutdown():
        # print(ideal_ugv.x[0, 0], ideal_ugv.x[1, 0], ideal_ugv.x[2, 0])
        if ideal_input is not None:
            if j != 0:
                ideal_ugv.astar_path.append([ideal_ugv.x, ideal_ugv.y, ideal_ugv.theta])
                faulty_ugv.astar_path.append([faulty_ugv.x, faulty_ugv.y, faulty_ugv.theta])

                # Compute Bicycle model equations
                iteration = current_index - start_index
                dt = rospy.get_time() - previous_time
                # ideal ugv dynamics
                ideal_ugv.dynamics(ideal_input.velocity, ideal_input.steering, ideal_condition, dt)
                ideal_pose.position.x = ideal_ugv.x
                ideal_pose.position.y = ideal_ugv.y
                ideal_pose.orientation.z = ideal_ugv.theta
                #faulty ugv dynamics
                faulty_ugv.dynamics(faulty_input.velocity, faulty_input.steering, fault_condition, dt)
                faulty_pose.position.x = faulty_ugv.x
                faulty_pose.position.y = faulty_ugv.y
                faulty_pose.orientation.x = fault_condition
                faulty_pose.orientation.y = target[0]
                faulty_pose.orientation.z = faulty_ugv.theta
                faulty_pose.orientation.w = target[1]
                j -= 1
            else:
                i -= 1
                target = ideal_ugv.path[i, :]
                j = ideal_ugv.iter

        if i == 0:
            ideal_pose.position.x = 123456789
            previous_time = rospy.get_time()
            agents_publisher.publish(msg)
            rate.sleep()

            ideal_ugv.path_data = np.asarray(ideal_ugv.path_data)
            ideal_ugv.astar_path = np.asarray(ideal_ugv.astar_path)
            faulty_ugv.astar_path = np.asarray(faulty_ugv.astar_path)
            plt.figure(figsize = (7,7))
            plt.plot(ideal_ugv.astar_path[:, 0], ideal_ugv.astar_path[:, 1], color='orange', linewidth=2)
            plt.plot(faulty_ugv.astar_path[:, 0], faulty_ugv.astar_path[:, 1], color='blue', linewidth=2)
            plt.plot(ideal_ugv.path[:,0], ideal_ugv.path[:,1], color='black', linestyle=':', linewidth=4)
            plt.xlabel('x')
            plt.ylabel('y')
            plt.title('UGV Path')
            plt.show()

            sys.exit(1)

        previous_time = rospy.get_time()
        agents_publisher.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    path = '/home/ace/catkin_ws/src/network_faults/data/path.csv'
    df = pd.read_csv(path)
    coordinates = df.to_numpy()

    ideal_condition = 3
    ideal_bicycle = Bicycle(coordinates)
    ideal_bicycle.createPath()

    fault_condition = 0
    fault_bicycle = Bicycle(coordinates)
    fault_bicycle.createPath()

    driveClosedLoop(ideal_bicycle, fault_bicycle, ideal_condition, fault_condition)
