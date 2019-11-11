import math
import numpy as np

class PID():
    def __init__(self, k):
        self.k = k
        self.v = 0
        self.g = 0
        self.sample_time = 0.001
        self.last_d = 0
        self.last_theta = 0
        self.sum_d = 0
        self.sum_theta = 0
        self.d_max = 1
        self.theta_max = math.pi
        self.fitness = 0

    def calculatePID(self, d, theta):
        self.sum_d = self.sum_d + d
        self.sum_d = np.clip(self.sum_d, -1*self.d_max, self.d_max)
        self.sum_theta = self.sum_theta + theta
        self.sum_theta = np.clip(self.sum_theta, -1*self.theta_max, self.theta_max)

        self.v = self.k[0]*d + self.k[1]*self.sum_d + self.k[2]*(d - self.last_d)*self.sample_time
        self.g = self.k[3]*theta + self.k[4]*self.sum_theta + self.k[5]*(theta - self.last_theta)*self.sample_time

        self.last_d = d
        self.last_theta = theta
