import math
import numpy as np

class PID():
    def __init__(self, fault):
        self.k = np.array([[4.77513214e-01, 7.21466250e-03, 2.71819786e+01, 3.72521390e+00, 2.31961582e+00, 1.64713994e+01]])
        self.velocity = 0
        self.steering = 0
        self.last_d = 0
        self.last_theta = 0
        self.Integral_d = 0
        self.Integral_theta = 0
        self.d_max = 1.0
        self.theta_max = math.radians(45)
        self.fitness = 0
        self.fault = fault

    def calculatePID(self, distance, heading, dt):
        self.Integral_d = self.Integral_d + distance
        self.Integral_d = np.clip(self.Integral_d, -1*self.d_max, self.d_max)
        self.Integral_theta = self.Integral_theta + heading
        self.Integral_theta = np.clip(self.Integral_theta, -1*self.theta_max, self.theta_max)

        self.velocity = self.k[self.fault, 0]*distance + self.k[self.fault, 1]*self.Integral_d + self.k[self.fault, 2]*(distance - self.last_d)*dt
        self.steering = self.k[self.fault, 3]*heading + self.k[self.fault, 4]*self.Integral_theta + self.k[self.fault, 5]*(heading - self.last_theta)*dt

        self.velocity = np.clip(self.velocity, -self.d_max, self.d_max)
        self.steering = np.clip(self.steering, -self.theta_max, self.theta_max)

        self.last_d = distance
        self.last_theta = heading

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

    def calculateError(self, target, x, y, theta):
        delta_x = np.clip(target[0] - x, -1e50, 1e50)
        delta_y = np.clip(target[1] - y, -1e50, 1e50)
        desired_heading = math.atan2(delta_y, delta_x)
        heading_error = self.angdiff(desired_heading, theta)

        delta_x2 = delta_x**2
        delta_y2 = delta_y**2
        if math.isinf(delta_x2):
            delta_x2 = 1e25
        if math.isinf(delta_y2):
            delta_y2 = 1e25

        distance_error = math.sqrt(delta_x2 + delta_y2)

        return distance_error, heading_error
