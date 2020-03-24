import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import numpy as np
from numpy.polynomial import Polynomial
import pandas as pd

def func(x, a, b, c):
    return a*np.exp(b*x)+c

data = pd.read_csv(r"C:\Users\conor\Documents\Code\network_faults\data\noise_data.csv")
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
