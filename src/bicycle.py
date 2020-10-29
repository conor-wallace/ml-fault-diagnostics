import math
import csv
import numpy as np
import matplotlib.pyplot as plt
from pid import PID
import pandas as pd

class Bicycle():
    def __init__(self, phi=0.0):
        self.x = 4.0
        self.y = 0.0
        self.theta = math.radians(90)
        self.drift_angle = phi
        self.desired_x = 0
        self.desired_y = 0
        self.desired_theta = 0
        self.heading_error = 0.0
        self.distance_error = 100000.0
        self.distance = 0.01
        self.k = [0.5, 0.0, 0.0, 1.0, 0.0, 0.0]
        self.T = 50.0
        self.max_rad = 38.0
        self.max_vel = 1.0
        self.max_iter = 1200*16
        self.L = 0.19
        self.iter = 20
        self.pid = PID(0)
        self.path_data = [[self.x, self.y, self.theta]]

    def dynamics(self, v, gamma, dt):
        self.x = np.clip(self.x, -1e5, 1e5)
        self.x = np.float64(self.x)

        if self.drift_angle == 0.0:
            # ideal dynamics
            yaw_dot = ((v/self.L)*(math.tan(gamma)))*dt
            x_dot = (v * math.cos(self.theta))*dt
            y_dot = (v * math.sin(self.theta))*dt

        else:
            # fault dynamics
            noise_gamma = gamma + self.drift_angle + np.random.normal(0, 0.01)
            yaw_dot = ((v/self.L)*(math.tan(noise_gamma)))*dt
            x_dot = (v * math.cos(self.theta))*dt
            y_dot = (v * math.sin(self.theta))*dt

        # yaw_dot = np.clip(yaw_dot, -math.radians(45), math.radians(45))
        # x_dot = np.clip(x_dot, -0.1, 0.1)
        # y_dot = np.clip(y_dot, -0.1, 0.1)

        self.theta = self.theta + yaw_dot
        self.x = self.x + x_dot
        self.y = self.y + y_dot

    def angdiff(self, a, b):
        diff = a - b
        if diff < 0.0:
            diff = (diff % (-2*math.pi))
            if diff < (-math.pi):
                diff = diff + 2*math.pi
        else:
            diff = (diff % 2*math.pi)
            if diff > math.pi:
                diff = diff - 2*math.pi

        return diff

    def calculate_error(self):
        delta_x = np.clip(self.desired_x - self.x, -1e50, 1e50)
        delta_y = np.clip(self.desired_y - self.y, -1e50, 1e50)
        desired_heading = math.atan2(delta_y, delta_x)
        self.heading_error = self.angdiff(desired_heading, self.theta)

        delta_x2 = delta_x**2
        delta_y2 = delta_y**2
        if math.isinf(delta_x2):
            delta_x2 = 1e25
        if math.isinf(delta_y2):
            delta_y2 = 1e25

        self.distance_error = math.sqrt(delta_x2 + delta_y2) - self.distance

    def drive_open_loop(self, fault, color):
        self.path_data = []
        i = 0

        while(i != self.max_iter):
            self.path_data.append([self.x, self.y])
            dt = (1.0/self.T)
            self.dynamics(0.36, -0.00023410732856159566, dt)
            i += 1

        path = np.asarray(self.path_data)
        #print(path.shape)
        plt.scatter(path[:, 0], path[:, 1], color=color)
        plt.xlabel("x (meters)")
        plt.ylabel("y (meters)")
        plt.title("UGV Path: Problem 1.a)")
        plt.show()

    def drive_along_path(self):
        # the target angle of the circular path being followed
        alpha = math.radians(0)
        cx = 2
        cy = 2
        r = 2
        i = 0
        self.desired_x = cx + r*math.cos(alpha)
        self.desired_y = cy + r*math.sin(alpha)

        while(i != self.max_iter):
            # Update trajectory
            if self.distance_error < self.distance*4:
                alpha += math.radians(2)
            self.desired_x = cx + r*math.cos(alpha)
            self.desired_y = cy + r*math.sin(alpha)

            # if i % 10 == 0:
            #     print("Desired X: ", self.desired_x)
            #     print("Desired Y: ", self.desired_y)
            #     print("Alpha: ", alpha)

            # Compute error
            self.calculate_error()
            # print("Heading error: ", self.heading_error)
            # print("Distance error: ", self.distance_error)

            # Compute controls
            dt = (1.0/self.T)
            self.pid.calculatePID(self.distance_error, self.heading_error, dt)

            # Execute controls
            self.dynamics(self.pid.velocity, self.pid.steering, dt)

            # Save new pose data and increment coutner
            i += 1
            self.path_data.append([self.x, self.y, self.theta])

        # Control scheme executed, save path data
        self.path_data = np.array(self.path_data)