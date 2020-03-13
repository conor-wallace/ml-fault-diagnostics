import math
import numpy as np

class PID():
    def __init__(self, k):
        self.k = k
        self.velocity = 0
        self.steering = 0
        self.last_d = 0
        self.last_theta = 0
        self.Integral_d = 0
        self.Integral_theta = 0
        self.d_max = 1.0
        self.theta_max = math.radians(45)
        self.fitness = 0

    def calculatePID(self, distance, heading, dt):
        self.Integral_d = self.Integral_d + distance
        self.Integral_d = np.clip(self.Integral_d, -1*self.d_max, self.d_max)
        self.Integral_theta = self.Integral_theta + heading
        self.Integral_theta = np.clip(self.Integral_theta, -1*self.theta_max, self.theta_max)

        self.velocity = self.k[0]*distance + self.k[1]*self.Integral_d + self.k[2]*(distance - self.last_d)*dt
        self.steering = self.k[3]*heading + self.k[4]*self.Integral_theta + self.k[5]*(heading - self.last_theta)*dt

        self.velocity = np.clip(self.velocity, -self.d_max, self.d_max)
        self.steering = np.clip(self.steering, -self.theta_max, self.theta_max)

        self.last_d = distance
        self.last_theta = heading
