import math
import csv
import numpy as np
import matplotlib.pyplot as plt
from pid import PID

class Rover:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.test_x = 0
        self.test_y = 0
        self.test_theta = 0
        self.theta = 0
        self.desired_x = 0
        self.desired_y = 0
        self.desired_theta = 0
        self.k = [0.1, 0.0, 0.0, 1, 0.0, 0.0]
        self.max_rad = 38.0
        self.max_vel = 1.0
        self.L = 0.19
        self.pid = PID(self.k)
        self.noise_path = []
        self.test_path = []
        self.prev_noise = 0.0
        self.noise_functions = np.array([[2.00000000e+01, 6.62201978e-03, -1.99941291e+01], [-5.10091508, -0.06503655, 5.08946158], [-0.01754703, 1.0, 0.06324038]])

    def dynamics(self, v, gamma, fault):
        #noise dynamics
        theta_dot = ((v/self.L)*(math.tan(gamma)))
        x_dot = v * math.cos(self.theta)
        y_dot = v * math.sin(self.theta)

        self.x = np.clip(self.x, -1e5, 1e5)
        self.x = np.float128(self.x)
        noise = self.noise_functions[fault, 0] * np.exp(self.noise_functions[fault, 1] * self.x) + self.noise_functions[fault, 2]
        #derivatives
        self.theta = self.theta + np.clip(theta_dot, -1*math.radians(self.max_rad), math.radians(self.max_rad))
        self.x = self.x + np.clip(x_dot, -1*self.max_vel, self.max_vel)
        self.y = self.y + np.clip(y_dot, -1*self.max_vel, self.max_vel)+(noise-self.prev_noise)
        self.prev_noise = noise

        #test dynamics
        theta_dot = ((v/self.L)*(math.tan(gamma)))
        x_dot = v * math.cos(self.test_theta)
        y_dot = v * math.sin(self.test_theta)

        self.x = np.clip(self.test_x, -1e5, 1e5)
        self.x = np.float128(self.test_x)
        #derivatives
        self.test_theta = self.test_theta + np.clip(theta_dot, -1*math.radians(self.max_rad), math.radians(self.max_rad))
        self.test_x = self.test_x + np.clip(x_dot, -1*self.max_vel, self.max_vel)
        self.test_y = self.test_y + np.clip(y_dot, -1*self.max_vel, self.max_vel)

    def driveAlongPath(self, fault):
        self.x = 0
        self.y = 0
        self.test_x = 0
        self.test_y = 0
        self.theta = math.radians(45)
        self.test_theta = math.radians(45)
        distance = 1000000.0
        prev_noise = 0.0

        self.desired_x = 10.0
        self.desired_y = 10.0
        delta_x = self.desired_x - self.x
        delta_y = self.desired_y - self.y

        while distance >= 0.5:
            test_delta_x = np.clip(self.test_desired_x - self.test_x, -1e50, 1e50)
            test_delta_y = np.clip(self.test_desired_y - self.test_y, -1e50, 1e50)
            self.test_desired_theta = math.atan2(test_delta_x, test_delta_y)

            test_delta_theta = self.test_desired_theta - self.test_theta
            test_delta_x2 = test_delta_x**2
            test_delta_y2 = test_delta_y**2
            if math.isinf(test_delta_x2):
                test_delta_x2 = 1e25
            if math.isinf(test_delta_y2):
                test_delta_y2 = 1e25
            test_distance = math.sqrt(test_delta_x2 + test_delta_y2)

            delta_x = np.clip(self.desired_x - self.x, -1e50, 1e50)
            delta_y = np.clip(self.desired_y - self.y, -1e50, 1e50)
            self.desired_theta = math.atan2(delta_y, delta_x)

            delta_theta = self.desired_theta - self.test_theta
            delta_x2 = delta_x**2
            delta_y2 = delta_y**2
            if math.isinf(delta_x2):
                delta_x2 = 1e25
            if math.isinf(delta_y2):
                delta_y2 = 1e25
            distance = math.sqrt(delta_x2 + delta_y2)

            self.noise_path.append([self.x, self.y])
            self.test_path.append([self.test_x, self.test_y])
            self.pid.calculatePID(distance, delta_theta)
            self.dynamics(self.pid.v, self.pid.g, fault)

        self.noise_path = np.asarray(self.noise_path)
        self.test_path = np.asarray(self.test_path)
        plt.plot(self.noise_path[:, 0], self.noise_path[:, 1], color='red', linewidth=2)
        plt.plot(self.test_path[:, 0], self.test_path[:, 1], color='blue', linewidth=2)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('UGV Path')
        plt.show()

if __name__ == '__main__':
    rover = Rover()
    rover.pid = PID([0.1, 0.0, 0.0, 2.0, 0.0, 0.0])
    rover.driveAlongPath(0)
