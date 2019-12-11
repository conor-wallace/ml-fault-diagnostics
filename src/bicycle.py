import math
import csv
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
        self.max_rad = 38.0
        self.max_vel = 1.0
        self.L = 0.19
        self.iter = 20
        self.pid = PID(self.k)
        self.desired_path = np.zeros(((self.path.shape[0])*self.iter,2))
        self.astar_path = []
        self.prev_noise = 0.0

    def createPath(self):
        x = self.path
        x = np.flip(x,0)
        print(x)
        plt.plot(x[:,0], x[:,1], 'o')
        for i in range(1,x.shape[0]):
            if x[i, 0]-x[i-1, 0] == 0.0:
                x_data = np.repeat(x[i,0], self.iter)
                y_data = np.linspace(start=x[i-1,1],stop=x[i,1], num=self.iter)
                print(x_data)
                print(y_data)
                plt.plot(x_data,y_data, '-x')
            else:
                a = (x[i, 1] - x[i-1, 1])/(x[i, 0]-x[i-1, 0])
                b = x[i, 1]-a*x[i, 0]
                print(a)
                print(b)
                x_data = np.linspace(start=x[i-1,0],stop=x[i,0], num=self.iter)
                y_data = list(map(lambda v : v * a + b, x_data))
                print(x_data)
                print(y_data)
                plt.plot(x_data,y_data, '-x')
            data = np.array(zip(x_data,y_data))
            print(data)
            print(data.shape)
            print(self.desired_path.shape)
            self.desired_path[i*self.iter:i*self.iter+self.iter,:] = data
        print(self.desired_path)
        plt.plot(self.desired_path[:,0], self.desired_path[:,1],color='blue')
        plt.show()

    def dynamics(self, v, gamma):
        #dynamics
        theta_dot = ((v/self.L)*(math.tan(gamma)))
        x_dot = v * math.cos(self.theta)
        y_dot = v * math.sin(self.theta)
        a = -0.494
        b = -0.487
        c = 0.488

        noise = a * np.exp(b * self.x) + c
        #derivatives
        self.theta = self.theta + np.clip(theta_dot, -1*math.radians(self.max_rad), math.radians(self.max_rad))
        self.x = self.x + np.clip(x_dot, -1*self.max_vel, self.max_vel)
        self.y = self.y + np.clip(y_dot, -1*self.max_vel, self.max_vel)+(noise-self.prev_noise)
        self.prev_noise = noise

    def driveAlongPath(self, index, pid, return_dict, plot):
        # print(pid.k)
        self.pid.k = pid.k
        i = len(self.path) - 2
        self.x = 0#path[i+1, 0]
        self.y = 0#path[i+1, 1]
        self.theta = math.radians(45)
        astar_path = []
        x_error = []
        y_error = []
        t_error = []
        d_error = []
        self.prev_noise = 0.0

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

                distance = np.clip(math.sqrt(delta_x**2 + delta_y**2), -1e50, 1e50)
                # print("theta error: %s" % delta_theta)
                # print("current [x, y]: [%s, %s]" % (self.x, self.y))
                # print("distance error: %s" % distance)
                x_error.append([delta_x])
                y_error.append([delta_y])
                t_error.append([delta_theta])
                d_error.append([distance])
                self.astar_path.append([self.x, self.y])
                self.pid.calculatePID(distance, delta_theta)
                self.dynamics(self.pid.v, self.pid.g)
                j = j - 1
            i = i - 1
        pid.fitness = self.objective()
        return_dict[index] = pid.fitness

        if plot:
            self.astar_path = np.asarray(self.astar_path)
            plt.plot(self.astar_path[:, 0], self.astar_path[:, 1], color='red')
            plt.plot(self.desired_path[:,0], self.desired_path[:,1], color='blue')
            plt.xlabel('x')
            plt.ylabel('y')
            plt.title('UGV Path')
            plt.show()

            time = np.arange(len(t_error))
            x_error = np.array(x_error)
            plt.plot(time[:], x_error[:, 0])
            plt.xlabel('t')
            plt.ylabel('x')
            plt.title('UGV X Trajectory')
            plt.show()

            y_error = np.array(y_error)
            plt.plot(time[:], y_error[:, 0])
            plt.xlabel('t')
            plt.ylabel('y')
            plt.title('UGV Y Trajectory')
            plt.show()

            time = np.arange(len(t_error))
            t_error = np.asarray(t_error)
            plt.plot(time[:], t_error[:, 0])
            plt.xlabel('t')
            plt.ylabel('theta')
            plt.title('UGV Theta Error')
            plt.show()

            d_error = np.asarray(d_error)
            plt.plot(time[:], d_error[:, 0])
            plt.xlabel('t')
            plt.ylabel('yl2 norm')
            plt.title('UGV Distance Error')
            plt.show()

            path_data = x_error
            path_data = np.concatenate((path_data, y_error), axis=1)
            path_data = np.concatenate((path_data, t_error), axis=1)
            path_data = np.concatenate((path_data, d_error), axis=1)
            print(path_data.shape)

            f=open("../data/ga_data.csv",'a')
            np.savetxt(f, path_data, delimiter=",")

    def objective(self):
        fitness = 0
        for i in range(len(self.astar_path)):
            fitness = fitness + np.linalg.norm(np.array(self.astar_path[i]) - np.array(self.desired_path[i]))

        return fitness
