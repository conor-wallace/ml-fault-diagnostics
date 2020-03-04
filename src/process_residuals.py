import numpy as np
import pandas as pd
import math
import sys
from matplotlib import pyplot as plt

#replace with the path on your pc
data = pd.read_csv("~/catkin_ws/src/network_faults/data/model_data.csv")
dataset = data.to_numpy()

split_data = np.empty((4, 400, 8))
count = 0
curr_index = 0
last_index = 0

for i in range(dataset.shape[0]):
    if dataset[i, 7] < last_index:
        count += 1
        curr_index = 0
        split_data[count, curr_index, :] = dataset[i, :]
        last_index = dataset[i, 7]
        curr_index += 1
    else:
        split_data[count, curr_index, :] = dataset[i, :]
        last_index = dataset[i, 7]
        curr_index += 1

model_data = np.empty(((split_data.shape[0]-1)*split_data.shape[1], 9))
for i in range(1, split_data.shape[0]):
    for j in range(split_data.shape[1]):
        model_data[((i-1)*split_data.shape[1] + j), :] = np.concatenate((split_data[i, j, :], i), axis=None)
        model_data[((i-1)*split_data.shape[1] + j), 1] -= split_data[0, j, 1]

plt.plot(model_data[0:400, 0], model_data[0:400, 1], color='green', label='Healthy Data')
plt.plot(model_data[400:800, 0], model_data[400:800, 1], color='red', label='Left Fault Data')
plt.plot(model_data[800:-1, 0], model_data[800:-1, 1], color='blue', label='Right Fault Data')
plt.legend()
plt.show()

f = open('/home/conor/catkin_ws/src/network_faults/data/residual_data.csv', 'a')
np.savetxt(f, model_data, delimiter=",")
