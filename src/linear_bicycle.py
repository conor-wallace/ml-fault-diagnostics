import math
import mpmath
import numpy as np
import matplotlib.pyplot as plt
from pid import PID

class LinearBicycle():
    def __init__(self, initial_conditions):
        self.x = np.array(initial_conditions)
        self.u = np.array([0.0, 0.0])
        self.T = 0.1
        self.desired_x = 0
        self.desired_y = 0
        self.desired_theta = 0
        self.k = [0.1, 0.0, 0.0, 1, 0.0, 0.0]
        self.max_rad = 38
        self.max_vel = 1
        self.L = 0.19
        self.iter = 20
        self.pid = PID(self.k)
        self.desired_path = []
        self.astar_path = []

    def dynamics(self):
        #dynamics
        A = [[1.0, 0.0, -self.u[0]*math.sin(self.x[2])*self.T],
             [0.0, 1.0, self.u[0]*math.cos(self.x[2])*self.T],
             [0.0, 0.0, 1.0]]
        B = [[math.cos(self.x[2])*self.T-self.u[0]*math.sin(self.x[2])*math.tan(self.u[1])*self.T*(1.0/2.0*self.L), -self.u[0]*math.sin(self.x[2])*((1/math.cos(self.u[1]))**2)*self.T*(1.0/2.0*self.L)],
             [math.sin(self.x[2])*self.T-self.u[0]*math.cos(self.x[2])*math.tan(self.u[1])*self.T*(1.0/2.0*self.L), -self.u[0]*math.cos(self.x[2])*((1/math.cos(self.u[1]))**2)*self.T*(1.0/2.0*self.L)],
             [math.tan(self.u[1])*(self.T/self.L), self.u[0]*((1/math.cos(self.u[1]))**2)*(self.T/self.L)]]

        A = np.array(A)
        B = np.array(B)

        self.x = np.reshape(self.x, (3, 1))
        self.u = np.reshape(self.u, (2, 1))
        x_dot = np.dot(A,self.x) + np.dot(B, self.u)
        x_dot = np.reshape(x_dot, (3, 1))

        for i in range(x_dot.shape[0]):
            x_dot[i] = float(x_dot[i])
        print(x_dot)

        self.x = self.x + x_dot
        print(self.x)

print(mpmath.sec(1))
print(1/math.cos(1))

x0 = [0,0,0]
model = LinearBicycle(x0)
model.u[0] = 1.0
model.u[1] = math.radians(38.0)
path = []
for i in range(20):
    model.dynamics()
    path.append([model.x[0], model.x[1]])

path = np.asarray(path)
plt.plot(path[:, 0], path[:, 1])
plt.xlabel('x')
plt.ylabel('y')
plt.title('UGV Path')
plt.show()
