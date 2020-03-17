import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

df = pd.read_csv("/home/conor/catkin_ws/src/network_faults/data/noise_functions.csv")
data = df[['%x1', '%x2', '%x3']]
data = data.to_numpy()
labels = df[['%label']]
labels = labels.to_numpy()

healthy = data[(labels==0).all(axis=1)]
left = data[(labels==1).all(axis=1)]
right = data[(labels==2).all(axis=1)]
print(healthy.shape)

# print(np.median(runtimes[(labels==0).all(axis=1)]))
# print(np.mean(runtimes[(labels==4).all(axis=1)]))
# print(np.mean(runtimes[(labels==1).all(axis=1)]))
# print(np.mean(runtimes[(labels==2).all(axis=1)]))
# print(np.mean(runtimes[(labels==3).all(axis=1)]))
# print(np.mean(runtimes[(labels==5).all(axis=1)]))

plt.hist(healthy[:, 1], color='green', label='Healthy x1 parameter')
plt.hist(left[:, 1], color='yellow', label='Left fault x1 parameter')
plt.hist(right[:, 1], color='red', label='Right fault x1 parameter')
plt.legend()
plt.title('LSTM vs. Real-Time Machine Learning Runtimes')
plt.xlabel('runtime (ms)')
plt.ylabel('number of runtimes')
# plt.ylim(0,50)
plt.show()
