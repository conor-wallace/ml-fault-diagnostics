import math
import numpy as np
import matplotlib.pyplot as plt
from pid import PID

class Bicycle():
    def __init__(self, path):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.path = np.dot(0.05, path)
        self.desired_x = 0
        self.desired_y = 0
        self.desired_theta = 0
        self.k = [0.1, 0.0, 0.0, 1, 0.0, 0.0]
        self.max_rad = 35
        self.max_vel = 2
        self.L = 0.019
        self.iter = 20
        self.pid = PID(self.k)
        self.desired_path = []
        self.astar_path = []

    def dynamics(self, v, gamma):
        #dynamics
        theta_dot = ((v/self.L)*(math.tan(gamma)))
        x_dot = v * math.cos(self.theta)
        y_dot = v * math.sin(self.theta)

        #derivatives
        self.theta = self.theta + np.clip(theta_dot, -1*math.radians(self.max_rad), math.radians(self.max_rad))
        self.x = self.x + np.clip(x_dot, -1*self.max_vel, self.max_vel)
        self.y = self.y + np.clip(y_dot, -1*self.max_vel, self.max_vel)

    def driveAlongPath(self):
        i = len(self.path) - 2
        self.x = 0#path[i+1, 0]
        self.y = 0#path[i+1, 1]
        self.theta = math.radians(45)
        astar_path = []
        t_error = []
        d_error = []

        while i >= 0:
            self.desired_x = self.path[i, 0]
            self.desired_y = self.path[i, 1]
            delta_x = self.desired_x - self.x
            delta_y = self.desired_y - self.y
            j = self.iter
            while j > 0:
                delta_x = self.desired_x - self.x
                delta_y = self.desired_y - self.y
                self.desired_theta = math.atan2(delta_y, delta_x)
                # print("current theta: %s" % self.desired_theta)
                # print("current heading: %s" % self.theta)
                delta_theta = self.desired_theta - self.theta

                distance = math.sqrt(delta_x**2 + delta_y**2)
                # print("theta error: %s" % delta_theta)
                # print("current [x, y]: [%s, %s]" % (self.x, self.y))
                # print("distance error: %s" % distance)
                t_error.append([delta_theta])
                d_error.append([distance])
                self.desired_path.append([self.desired_x, self.desired_y, self.desired_theta])
                self.astar_path.append([self.x, self.y, self.theta])
                self.pid.calculatePID(distance, delta_theta)
                self.dynamics(self.pid.v, self.pid.g)
                j = j - 1
            i = i - 1
        #
        # astar_path = np.asarray(astar_path)
        # plt.plot(astar_path[:, 0], astar_path[:, 1])
        # plt.xlabel('x')
        # plt.ylabel('y')
        # plt.title('UGV Path')
        # plt.show()
        #
        # time = np.arange(len(t_error))
        # t_error = np.asarray(t_error)
        # plt.plot(time[:], t_error[:, 0])
        # plt.xlabel('t')
        # plt.ylabel('theta')
        # plt.title('UGV Theta Error')
        # plt.show()
        #
        # d_error = np.asarray(d_error)
        # plt.plot(time[:], d_error[:, 0])
        # plt.xlabel('t')
        # plt.ylabel('yl2 norm')
        # plt.title('UGV Distance Error')
        # plt.show()

    def objective(self):
        fitness = 0
        for i in range(len(self.astar_path)):
            fitness = fitness + np.linalg.norm(np.array(self.astar_path[i]) - np.array(self.desired_path[i]))**2

        return fitness
