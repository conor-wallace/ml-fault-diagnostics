import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import numpy as np
from numpy.polynomial import Polynomial
import pandas as pd
import math

def dynamics(x, y, theta, v, gamma, dt):
    yaw_dot = ((v/0.19)*(math.tan(gamma)))*dt
    x_dot = (v * math.cos(theta))*dt
    y_dot = (v * math.sin(theta))*dt

    theta = theta + yaw_dot
    x = x + x_dot
    y = y + y_dot

    return x, y, theta

def driveOpenLoop(gamma, max_iter):
    y_data = []
    i = 0
    x, y, theta = 0.0, 0.0, math.radians(0.0)

    while(i != max_iter):
        y_data.append(y)
        x, y, theta = dynamics(x, y, theta, 0.4, gamma, 0.1)
        i += 1

    return np.asarray(y_data)

def func(x, gamma):
    y = driveOpenLoop(gamma, x.shape[0])

    return y

data = pd.read_csv("/home/conor/catkin_ws/src/network_faults/data/noise_data.csv")
dataset = data.to_numpy()

healthy_data = dataset[dataset[:,-1]==0]
left_data = dataset[dataset[:,-1]==1]
right_data = dataset[dataset[:,-1]==2]

plt.scatter(healthy_data[:,0], healthy_data[:,1], color='#91bfdb', marker='.', label='healthy data')
plt.scatter(left_data[:,0], left_data[:,1], color='#FFC39D', marker='.', label='left fault')
plt.scatter(right_data[:,0], right_data[:,1], color='#99d594', marker='.', label='right fault')
plt.title("Fault Trajectory Data")
plt.xlabel("x (meters)")
plt.ylabel("y (meters)")
plt.legend()
plt.show()

dataset = np.reshape(dataset, (-1, 270, 4))
noise_functions = []

for i in range(60):
    color = 'black'
    xdata = dataset[i,:,0]
    y = dataset[i,:,1]
    label = dataset[i,0,-1]
    ydata = y
    plt.scatter(xdata, ydata, color='#91bfdb', marker='.', label='healthy data')
    plt.ylabel('y (meters)')
    plt.xlabel('x (meters)')
    plt.legend()
    plt.title('Healthy Trajectory')
    # plt.show()

    plt.scatter(xdata, ydata, color='#91bfdb', marker='.', label='healthy data')

    popt, pcov = curve_fit(func, xdata, ydata, bounds=([-math.radians(38)], [math.radians(38)]))
    print(popt)

    noise_functions.append([popt, label])

    plt.scatter(xdata, func(xdata, *popt), color=color, marker='p')

    plt.ylabel('y (meters)')
    plt.xlabel('x (meters)')
    plt.legend()
    plt.xlim(0, 3)
    plt.title('Healthy Trajectory')
    # plt.show()

noise_functions = np.array(noise_functions)

f = open('/home/conor/catkin_ws/src/network_faults/data/noise_functions.csv', 'a')
np.savetxt(f, noise_functions, delimiter=",")

healthy_functions = noise_functions[noise_functions[:, -1]==0]
left_functions = noise_functions[noise_functions[:, -1]==1]
right_functions = noise_functions[noise_functions[:, -1]==2]

healthy_functions = np.reshape(healthy_functions, (20, -1))
left_functions = np.reshape(left_functions, (20, -1))
right_functions = np.reshape(right_functions, (20, -1))

distribution_data = np.empty((3, 3))
distribution_data[0, :] = [np.mean(healthy_functions[:, 0]), np.std(healthy_functions[:, 0]), 0]
distribution_data[1, :] = [np.mean(left_functions[:, 0]), np.std(left_functions[:, 0]), 1]
distribution_data[2, :] = [np.mean(right_functions[:, 0]), np.std(right_functions[:, 0]), 2]

f = open('/home/conor/catkin_ws/src/network_faults/data/distributions.csv', 'a')
np.savetxt(f, distribution_data, delimiter=",")

# f, (ax1, ax2, ax3) = plt.subplots(1, 3)
# color = 'black'
# xdata = healthy_data[:,0]
# print(xdata)
# y = healthy_data[:,1]
# ydata = y
# ax1.scatter(xdata, ydata, color='#91bfdb', marker='.', label='healthy data')
#
# popt, pcov = curve_fit(func, xdata, ydata, bounds=([-20, -1.0, -100], [500, 10.0, 20]))
# print(popt)
#
# ax1.scatter(xdata, func(xdata, *popt), color=color, marker='p')
#
# ax1.set_ylabel('y (meters)')
# ax1.set_xlabel('x (meters)')
# ax1.set_xlim(-0.1,2.25)
# ax1.set_ylim(-0.4,0.25)
# ax1.legend()
# ax1.set_title('Healthy Trajectory')
#
# xdata = left_data[:,0]
# y = left_data[:,1]
# ydata = y
# ax2.scatter(xdata, ydata, color='#FFC39D', marker='.', label='left fault')
#
# popt, pcov = curve_fit(func, xdata, ydata, bounds=([-20, -1.0, -20], [20, 10.0, 20]))
# print(popt)
#
# ax2.scatter(xdata, func(xdata, *popt), color=color, marker='p')
#
# ax2.set_xlabel('x (meters)')
# ax2.set_xlim(-0.1,2.25)
# ax2.set_ylim(-0.4,0.25)
# ax2.legend()
# ax2.set_title('Left Fault Trajectory')
#
#
#
# xdata = right_data[:,0]
# y = right_data[:,1]
# ydata = y
# ax3.scatter(xdata, ydata, color='#99d594', marker='.', label='right fault')
#
# popt, pcov = curve_fit(func, xdata, ydata, method='trf', bounds=([-40, -1.0, -20], [20, 10.0, 20]))
# print(popt)
#
# ax3.scatter(xdata, func(xdata, *popt), color=color, marker='p')
#
# ax3.set_xlabel('x (meters)')
# ax3.set_xlim(-0.1,2.25)
# ax3.set_ylim(-0.4,0.25)
# ax3.legend()
# ax3.set_title('Right Fault Trajectory')
#
# plt.show()
