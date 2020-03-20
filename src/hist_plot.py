import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import math

def normal(x, mu, sigma):
    print(x.shape)
    return (1.0/(sigma*np.sqrt(2*np.pi)))*(np.exp(-((x-mu)/sigma)**2/2.0))

df1 = pd.read_csv("/home/conor/catkin_ws/src/network_faults/data/noise_functions.csv")
noise_data = df1[['%x1', '%x2', '%x3']]
noise_data = noise_data.to_numpy()
labels = df1[['%label']]
labels = labels.to_numpy()

healthy_function = noise_data[(labels==0).all(axis=1)]
left_function = noise_data[(labels==1).all(axis=1)]
right_function = noise_data[(labels==2).all(axis=1)]
print(healthy_function.shape)

index = 2

count, healthy_bins, ignored = plt.hist(healthy_function[:, index], color='#91bfdb', label='Healthy x3 parameter', density=True)
count, left_bins, ignored = plt.hist(left_function[:, index], color='#FFC39D', label='Left fault x3 parameter')
count, right_bins, ignored = plt.hist(right_function[:, index], color='#99d594', label='Right fault x3 parameter')
plt.legend()
plt.title('Noise Function Parameter X3 Distributions')
plt.xlabel('parameter value')
plt.ylabel('number of occurences of values')
# plt.ylim(0,50)
# plt.show()

# count, bins, ignored = plt.hist(s, 30, density=True)

df2 = pd.read_csv("/home/conor/catkin_ws/src/network_faults/data/distributions.csv")
dist_data = df2[['%mean1','%mean2','%mean3','%var1','%var2','%var3','%label']]
dist_data = dist_data.to_numpy()
dist_labels = df2[['%label']]
dist_labels = dist_labels.to_numpy()

healthy_dist = dist_data[dist_data[:, -1]==0]
left_dist = dist_data[dist_data[:, -1]==1]
right_dist = dist_data[dist_data[:, -1]==2]
print(healthy_dist.shape)

healthy_bins = np.array(healthy_bins)
healthy_sigma = float(healthy_dist[0, index+3])
healthy_mu = float(healthy_dist[0, index])

left_bins = np.array(left_bins)
left_sigma = float(left_dist[0, index+3])
left_mu = float(left_dist[0, index])

right_bins = np.array(right_bins)
right_sigma = float(right_dist[0, index+3])
right_mu = float(right_dist[0, index])

print(healthy_bins)

ranges = np.array([[-0.12, 0.35],
                   [0, 1.3],
                   [-0.32, 0.13]])

x = np.arange(ranges[index, 0], ranges[index, 1], 0.005)

print(x)

plt.plot(x, normal(x, healthy_mu, healthy_sigma), linewidth=2, linestyle='dashed', color='#91bfdb')
plt.plot(x, normal(x, left_mu, left_sigma), linewidth=2, linestyle='dashed', color='#FFC39D')
plt.plot(x, normal(x, right_mu, right_sigma), linewidth=2, linestyle='dashed', color='#99d594')
plt.show()
