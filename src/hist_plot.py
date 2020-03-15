import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

df = pd.read_csv("/home/conor/catkin_ws/src/network_faults/data/runtime.csv")
runtimes = df[['%runtime']]
runtimes = runtimes.to_numpy()
labels = df[['%label']]
labels = labels.to_numpy()

print(np.median(runtimes[(labels==0).all(axis=1)]))
print(np.mean(runtimes[(labels==4).all(axis=1)]))
print(np.mean(runtimes[(labels==1).all(axis=1)]))
print(np.mean(runtimes[(labels==2).all(axis=1)]))
print(np.mean(runtimes[(labels==3).all(axis=1)]))
print(np.mean(runtimes[(labels==5).all(axis=1)]))

plt.hist(runtimes[(labels==0).all(axis=1)], bins=900, range=(0.1, 4), color='green', label='LSTM = 3.890')
plt.hist(runtimes[(labels==4).all(axis=1)], bins=900, range=(0.1, 4), color='yellow', label='LR = 0.222')
plt.hist(runtimes[(labels==1).all(axis=1)], bins=900, range=(0.1, 4), color='red', label='QDA = 0.545')
plt.hist(runtimes[(labels==2).all(axis=1)], bins=900, range=(0.1, 4), color='purple', label='LDA = 0.224')
plt.hist(runtimes[(labels==3).all(axis=1)], bins=900, range=(0.1, 4), color='orange', label='KNN = 2.090')
plt.hist(runtimes[(labels==5).all(axis=1)], bins=900, range=(0.1, 4), color='blue', label='SVM = 2.142')
plt.legend()
plt.title('LSTM vs. Real-Time Machine Learning Runtimes')
plt.xlabel('runtime (ms)')
plt.ylabel('number of runtimes')
plt.ylim(0,50)
plt.show()
