import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import numpy as np
from numpy.polynomial import Polynomial
import pandas as pd

def func(x, a, b, c):
    return a * np.exp(b * x) + c

data = pd.read_csv("~/catkin_ws/src/network_faults/data/lilbot_data.csv")
dataset = data[['x', 'y']]
dataset = dataset.to_numpy()
healthy_data = dataset[0:2250]
left_data = dataset[2250:4500]
right_data = dataset[4500:-1]

xdata = healthy_data[:,0]
y = healthy_data[:,1]
ydata = y
plt.scatter(xdata, ydata, c='blue', marker='.', label='data')

popt, pcov = curve_fit(func, xdata, ydata, bounds=([-20, -1.0, -20], [20, 1.0, 20]))
popt

plt.scatter(xdata, func(xdata, *popt), c='red', marker='p',
         label='constrained fit: a=%5.3f, b=%5.3f, c=%5.3f' % tuple(popt))

plt.xlabel('x')
plt.ylabel('y')
plt.title('Healthy Data')
plt.legend()
plt.show()

xdata = left_data[:,0]
y = left_data[:,1]
ydata = y
plt.scatter(xdata, ydata, c='blue', marker='.', label='data')

popt, pcov = curve_fit(func, xdata, ydata, bounds=([-20, -1.0, -20], [20, 1.0, 20]))
popt

plt.scatter(xdata, func(xdata, *popt), c='red', marker='p',
         label='constrained fit: a=%5.3f, b=%5.3f, c=%5.3f' % tuple(popt))

plt.xlabel('x')
plt.ylabel('y')
plt.title('Left Fault Data')
plt.legend()
plt.show()



xdata = right_data[:,0]
y = right_data[:,1]
ydata = y
plt.scatter(xdata, ydata, c='blue', marker='.', label='data')

popt, pcov = curve_fit(func, xdata, ydata, bounds=([-20, -1.0, -20], [20, 1.0, 20]))
popt

plt.scatter(xdata, func(xdata, *popt), c='red', marker='p',
         label='constrained fit: a=%5.3f, b=%5.3f, c=%5.3f' % tuple(popt))

plt.xlabel('x')
plt.ylabel('y')
plt.title('Right Fault Data')
plt.legend()
plt.show()
