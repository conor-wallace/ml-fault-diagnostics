import numpy as np
import pandas as pd
import math
import sys
from matplotlib import pyplot as plt

data = pd.read_csv("~/catkin_ws/src/network_faults/data/lilbot_data.csv")
dataset = data[['iteration','x', 'y']]
dataset = dataset.to_numpy()
healthy_data = dataset[0:2250]
left_data = dataset[2250:4500]
right_data = dataset[4500:-1]

print(dataset.shape)

plt.scatter(healthy_data[:, 1], healthy_data[:, 2], color='blue')
plt.scatter(left_data[:, 1], left_data[:, 2], color='red')
plt.scatter(right_data[:, 1], right_data[:, 2], color='orange')
plt.xlabel('x')
plt.ylabel('y')
plt.ylim(-1,1)
plt.title('Healthy Path Data')
plt.show()
