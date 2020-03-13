import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import numpy as np
from numpy.polynomial import Polynomial
import pandas as pd

def func(x, a, b, c):
    return a / (1 + np.exp(-b * (x+c)))

data = pd.read_csv("~/catkin_ws/src/network_faults/data/lilbot_data.csv")
dataset = data.to_numpy()
healthy_data = dataset[0:1800]
left_data = dataset[1800:3600]
right_data = dataset[3600:-1]

f, (ax1, ax2, ax3) = plt.subplots(1, 3)
color = 'black'
xdata = healthy_data[:,2]
print(xdata)
y = healthy_data[:,3]
ydata = y
ax1.scatter(xdata, ydata, color='#91bfdb', marker='.')

popt, pcov = curve_fit(func, xdata, ydata, bounds=([-20, -1.0, -20], [20, 10.0, 20]))
print(popt)

ax1.scatter(xdata, func(xdata, *popt), color=color, marker='p')

ax1.set_ylabel('y')
ax1.set_xlabel('x')
ax1.set_xlim(-0.1,3.1)
ax1.set_ylim(-0.6,0.95)
ax1.legend()
ax1.set_title('Healthy Trajectory')

xdata = left_data[:,2]
y = left_data[:,3]
ydata = y
ax2.scatter(xdata, ydata, color='#FFC39D', marker='.')

popt, pcov = curve_fit(func, xdata, ydata, bounds=([-20, -1.0, -20], [20, 10.0, 20]))
print(popt)

ax2.scatter(xdata, func(xdata, *popt), color=color, marker='p')

ax2.set_xlabel('x')
ax2.set_xlim(-0.1,3.1)
ax2.set_ylim(-0.6,0.95)
ax2.legend()
ax2.set_title('Left Fault Trajectory')



xdata = right_data[:,2]
y = right_data[:,3]
ydata = y
ax3.scatter(xdata, ydata, color='#99d594', marker='.')

popt, pcov = curve_fit(func, xdata, ydata, method='trf', bounds=([-40, -1.0, -20], [20, 10.0, 20]))
print(popt)

ax3.scatter(xdata, func(xdata, *popt), color=color, marker='p')

ax3.set_xlabel('x')
ax3.set_xlim(-0.1,3.1)
ax3.set_ylim(-0.6,0.95)
ax3.legend()
ax3.set_title('Right Fault Trajectory')

plt.show()
