import numpy as np
import pandas as pd
import math
import sys
from matplotlib import pyplot as plt

#replace with the path on your pc
data = pd.read_csv("~/catkin_ws/src/network_faults/data/lilbot_data.csv")
dataset = data[['iteration','x', 'y']]
dataset = dataset.to_numpy()
healthy_data = dataset[0:1800]
left_data = dataset[1800:3600]
right_data = dataset[3599:-1]

plt.scatter(healthy_data[:, 1], healthy_data[:, 2], color='green', label='Healthy')
plt.scatter(left_data[:, 1], left_data[:, 2], color='red', label='Left Suspension Fault')
plt.scatter(right_data[:, 1], right_data[:, 2], color='blue', label='Right Suspension Fault')
plt.title("Fault Trajectories")
plt.xlabel("x")
plt.ylabel("y")
plt.legend()
plt.show()

print(right_data.shape)

curr_index = 0
sequences = 447

# f, (ax1, ax2, ax3) = plt.subplots(1, 3)
# ax1.scatter(healthy_data[:, 1], healthy_data[:, 2], color='#91bfdb', label='no fault')
# ax2.scatter(left_data[:, 1], left_data[:, 2], color='#FFC39D', label='left fault')
# ax3.scatter(right_data[:, 1], right_data[:, 2], color='#99d594', label='right fault')
# ax1.set_xlim(-0.1,3.1)
# ax1.set_ylim(-0.6,0.95)
# ax1.set_xlabel('x')
# ax1.set_ylabel('y')
# ax1.legend()
# ax1.set_title('Healthy Trajectory')
# ax2.set_xlim(-0.1,3.1)
# ax2.set_ylim(-0.6,0.95)
# ax2.set_xlabel('x')
# ax2.legend()
# ax2.set_title('Left Fault Trajectory')
# ax3.set_xlim(-0.1,3.1)
# ax3.set_ylim(-0.6,0.95)
# ax3.set_xlabel('x')
# ax3.legend()
# ax3.set_title('Right Fault Trajectory')
# plt.show()
