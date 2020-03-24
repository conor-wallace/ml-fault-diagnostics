from bicycle import Bicycle
from pid import PID
import numpy as np
import matplotlib.pyplot as plt
import math

def drive_to_target(ugv, target, fault):
    pid = PID(0)
    ugv.theta = math.radians(0)
    i = 0
    distance = 100
    start_point = np.array([ugv.x, ugv.y, ugv.theta])
    max_iter = 270
    steering_curr = 0.0
    velocity = 0.4
    steering_next = 0.0
    velocity_next = 0.0
    y = np.empty((max_iter, 2))
    y_dot = np.empty((max_iter, 2))

    while(i != max_iter):
        ugv.path_data.append([ugv.x, ugv.y, ugv.theta, i])
        ugv.calculateError(target)
        pid.calculatePID(ugv.distance_error, ugv.heading_error, 0.1)
        # print("GAMMA: %s" % pid.steering)
        steering = pid.steering
        velocity_next = pid.velocity
        y[i, :] = [ugv.y + velocity*math.sin(ugv.theta), i]
        ugv.dynamics(velocity, 0.0, fault, 0.1)
        y_dot[i, :] = [ugv.y, i]
        i += 1

    path = np.array(ugv.path_data)
    print(path.shape)
    plt.plot(path[:, 0], path[:, 1])
    plt.xlabel("x (meters)")
    plt.ylabel("y (meters)")
    plt.title("UGV Path: Problem 1.b)")
    # plt.gcf().gca().add_artist(Wedge(center=(int(start_point[0]), int(start_point[1])), r=1, theta1=45, theta2=75, width=1))
    plt.scatter(start_point[0], start_point[1], marker='o', color='blue')
    plt.show()

    # print(path)
    # print(y_dot)
    noise_data = np.subtract(y_dot[:, 0], y[:, 0])
    plt.plot(path[:, 0], noise_data)
    plt.show()

if __name__ == '__main__':
    path = np.array([1.0, 1.0])
    ugv = Bicycle(path)
    target = np.array([2.0, 2.0])

    drive_to_target(ugv, target, 0)
