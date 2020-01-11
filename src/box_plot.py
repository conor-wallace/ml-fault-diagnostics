import numpy as np
import pandas as pd
import math
import sys
from matplotlib import pyplot as plt

data = pd.read_csv("~/catkin_ws/src/network_faults/data/nn_data.csv")
data = data[['%tacc1','%tacc2','%tacc3','%tacc4','%tacc5']]
data = data.to_numpy()
collectn_1 = data[0, :]
collectn_2 = data[1, :]
collectn_3 = data[2, :]
collectn_4 = data[3, :]
collectn_5 = data[4, :]

## combine these different collections into a list
data_to_plot = [collectn_1, collectn_2, collectn_3, collectn_4, collectn_5]

# Create a figure instance
fig = plt.figure(1, figsize=(9, 6))

# Create an axes instance
ax = fig.add_subplot(111)

# Create the boxplot
bp = ax.boxplot(data_to_plot)

## Custom x-axis labels
ax.set_xticklabels(['LSTM', 'Multichannel CNN', 'Multiheaded CNN', 'CNN-LSTM', 'ConvLSTM'])
ax.set_ylim(96,100)
ax.set_title('Neural Network Training Accuracy')
plt.ylabel('Percent Accuracy')
plt.show()
