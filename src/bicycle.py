import math
import csv
import numpy as np
import matplotlib.pyplot as plt
from pid import PID
import pandas as pd

class Bicycle():
    def __init__(self, path):
        self.x = 0
        self.y = 0
        self.theta = math.radians(0)
        self.path = path
        self.desired_x = 0
        self.desired_y = 0
        self.desired_theta = 0
        self.heading_error = 0.0
        self.distance_error = 100000.0
        self.k = [0.5, 0.01, 0.01, 1.0, 0.01, 0.01]
        self.max_rad = 38.0
        self.max_vel = 1.0
        self.max_iter = 45
        self.L = 0.19
        self.iter = 20
        self.pid = PID(0)
        self.desired_path = np.zeros(((self.path.shape[0])*self.iter,2))
        self.astar_path = []
        self.path_data = []
        self.prev_noise = 0.0
        self.noise_distribution = None
        self.noise_gamma = 0.0
        self.white_noise = 0.0

    def createPath(self):
        C = np.zeros((40, 40))
        wall1 = [[20,30],[21,30],[22,30],[23,30],[24,30],[25,30],[26,30],[27,30],[28,30],[29,30],[30,30]]
        wall1 = np.reshape(wall1, (11, 2))
        wall2 = np.flip(wall1, 1)

        wall1_count = 0
        wall2_count = 0
        # path_count = len(path) - 1
        #print(path[0])

        for i in range(C.shape[0]):
            for j in range(C.shape[1]):
                if wall1_count <= 10:
                    if i == wall1[wall1_count, 0] and j == wall1[wall1_count, 1]:
                        C[i, j] = 1
                        wall1_count += 1
                if wall2_count <= 10:
                    if i == wall2[wall2_count, 0] and j == wall2[wall2_count, 1]:
                        C[i, j] = 1
                        wall2_count += 1
                coordinate = [i, j]
                # if coordinate in path:
                #     print(coordinate)
                #     C[i, j] = 10
                #     path_count -= 1
        plt.imshow(C, cmap='Greys')
        plt.gca().invert_yaxis()
        plt.ylabel("y (meters)")
        plt.xlabel("x (meters)")
        plt.title("A* Optimal Path")
        x = self.path
        x = np.flip(x,0)
        plt.plot(x[:,0], x[:,1], 'o')
        for i in range(1,x.shape[0]):
            if x[i, 0]-x[i-1, 0] == 0.0:
                x_data = np.repeat(x[i,0], self.iter)
                y_data = np.linspace(start=x[i-1,1],stop=x[i,1], num=self.iter)
                #print(x_data)
                #print(y_data)
                plt.plot(x_data,y_data, '-x')
            else:
                a = (x[i, 1] - x[i-1, 1])/(x[i, 0]-x[i-1, 0])
                b = x[i, 1]-a*x[i, 0]
                #print(a)
                #print(b)
                x_data = np.linspace(start=x[i-1,0],stop=x[i,0], num=self.iter)
                y_data = list(map(lambda v : v * a + b, x_data))
                #print(x_data)
                #print(y_data)
                plt.plot(x_data,y_data, '-x')
            data = np.array(zip(x_data,y_data))
            #print(data)
            #print(data.shape)
            #print(self.desired_path.shape)
            self.desired_path[i*self.iter:i*self.iter+self.iter,:] = data
        scale = float(3.0)/float(35.0)
        self.path = self.path*scale
        self.desired_path = self.desired_path*scale
        thetas = [math.radians(45)]
        for i in range(1,self.desired_path.shape[0]):
            delta_x = self.desired_path[i, 0] - self.desired_path[i-1, 0]
            delta_y = self.desired_path[i, 1] - self.desired_path[i-1, 1]
            desired_heading = math.atan2(delta_y, delta_x)
            thetas.append(desired_heading)
        thetas = np.array(thetas)
        thetas = np.reshape(thetas, (-1, 1))

        self.desired_path = np.concatenate((self.desired_path, thetas), axis=1)
        #print(self.desired_path)
        plt.plot(self.desired_path[:,0], self.desired_path[:,1],color='blue')
        plt.show()

    def readNoiseFunction(self):
        path = "/home/conor/catkin_ws/src/network_faults/data/distributions.csv"
        df = pd.read_csv(path)
        dataset = df[['%mean','%var']]
        dataset = dataset.to_numpy()

        dataset = np.concatenate([np.reshape(np.zeros(dataset.shape[1]), (1, -1)), dataset])
        print(dataset)

        self.noise_distribution = dataset

    def setNoiseFunction(self, fault):
        gamma_mu = self.noise_distribution[fault, 0]
        gamma_sigma = self.noise_distribution[fault, 1]

        print(self.noise_distribution[fault, :])

        self.noise_gamma = np.random.normal(gamma_mu*30, gamma_sigma**2, 1)

        print(self.noise_gamma)

        if fault != 0:
            self.white_noise = 0.0

    def dynamics(self, v, gamma, fault, dt):
        self.x = np.clip(self.x, -1e5, 1e5)
        self.x = np.float128(self.x)

        if fault == 0:
            # ideal dynamics
            yaw_dot = ((v/self.L)*(math.tan(gamma)))*dt
            x_dot = (v * math.cos(self.theta))*dt
            y_dot = (v * math.sin(self.theta))*dt

        else:
            # fault dynamics
            noise_gamma = gamma + self.noise_gamma + np.random.normal(0, self.white_noise)
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

    def calculateError(self, target):
        delta_x = np.clip(target[0] - self.x, -1e50, 1e50)
        delta_y = np.clip(target[1] - self.y, -1e50, 1e50)
        desired_heading = math.atan2(delta_y, delta_x)
        self.heading_error = self.angdiff(desired_heading, self.theta)

        delta_x2 = delta_x**2
        delta_y2 = delta_y**2
        if math.isinf(delta_x2):
            delta_x2 = 1e25
        if math.isinf(delta_y2):
            delta_y2 = 1e25

        self.distance_error = math.sqrt(delta_x2 + delta_y2)

    # def calculateError(self, target):
    #     delta_x = np.clip(target[0] - self.x, -1e50, 1e50)
    #     delta_y = np.clip(target[1] - self.y, -1e50, 1e50)
    #     desired_heading = math.atan2(delta_y, delta_x)
    #
    #     self.heading_error = desired_heading - self.theta
    #     delta_x2 = delta_x**2
    #     delta_y2 = delta_y**2
    #     if math.isinf(delta_x2):
    #         delta_x2 = 1e25
    #     if math.isinf(delta_y2):
    #         delta_y2 = 1e25
    #     self.distance_error = math.sqrt(delta_x2 + delta_y2)

    def driveOpenLoop(self, fault, color):
        self.path_data = []
        i = 0

        while(i != self.max_iter):
            self.path_data.append([self.x, self.y])
            self.dynamics(0.4, 0.0, fault, 0.1)
            i += 1

        path = np.asarray(self.path_data)
        #print(path.shape)
        plt.scatter(path[:, 0], path[:, 1], color=color)
        plt.xlabel("x (meters)")
        plt.ylabel("y (meters)")
        plt.title("UGV Path: Problem 1.a)")
        plt.show()

    def driveAlongPath(self, index, pid, return_dict, plot, fault):
        self.pid.k = pid.k
        i = len(self.path) - 2
        self.x = 0
        self.y = 0
        self.theta = math.radians(45)
        astar_path = []
        x_error = []
        y_error = []
        t_error = []
        d_error = []
        self.prev_noise = 0.0
        self.path_data = []
        self.astar_path = []
        count = 0

        while i >= 0:
            target = self.path[i, :]
            self.desired_y = self.path[i, 1]
            delta_x = self.desired_x - self.x
            delta_y = self.desired_y - self.y
            j = self.iter
            while j > 0:
                self.calculateError(target)

                # x_error.append([delta_x])
                # y_error.append([delta_y])
                # t_error.append([delta_theta])
                # d_error.append([distance])
                self.astar_path.append([self.x, self.y, self.theta])
                self.path_data.append([self.x, self.y, self.theta, self.pid.velocity, self.pid.steering, count, fault])
                count += 1
                self.pid.calculatePID(self.distance_error, self.heading_error, 0.1)
                self.dynamics(self.pid.velocity, self.pid.steering, fault, 0.1)
                j = j - 1
            i = i - 1
        pid.fitness = self.objective()
        # print("fitness: %s" % pid.fitness)
        if return_dict is not None:
            return_dict[index] = pid.fitness

        if plot:
            self.path_data = np.asarray(self.path_data)
            self.astar_path = np.asarray(self.astar_path)
            plt.figure(figsize = (7,7))
            plt.plot(self.astar_path[:, 0], self.astar_path[:, 1], color='orange', linewidth=2)
            plt.plot(self.path[:,0], self.path[:,1], color='blue', linestyle=':', linewidth=4)
            plt.xlabel('x')
            plt.ylabel('y')
            plt.title('UGV Path')
            plt.show()

            # time = np.arange(len(t_error))
            # x_error = np.array(x_error)
            # plt.plot(time[:], x_error[:, 0])
            # plt.xlabel('t')
            # plt.ylabel('x')
            # plt.title('UGV X Trajectory')
            # plt.show()
            #
            # y_error = np.array(y_error)
            # plt.plot(time[:], y_error[:, 0])
            # plt.xlabel('t')
            # plt.ylabel('y')
            # plt.title('UGV Y Trajectory')
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

            # path_data = x_error
            # path_data = np.concatenate((path_data, y_error), axis=1)
            # path_data = np.concatenate((path_data, t_error), axis=1)
            # path_data = np.concatenate((path_data, d_error), axis=1)
            # print(path_data.shape)
            #
            # f=open("../data/ga_data.csv",'a')
            # np.savetxt(f, path_data, delimiter=",")

    def objective(self):
        # fitness = 0
        self.astar_path = np.array(self.astar_path)
        fitness = np.linalg.norm(self.desired_path[20:, :] - self.astar_path)**2
        # for i in range(len(self.astar_path)):
        #     fitness = fitness + np.linalg.norm(np.array(self.astar_path[i]) - np.array(self.desired_path[i]))

        return fitness
