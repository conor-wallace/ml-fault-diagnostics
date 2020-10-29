import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import numpy as np
from numpy.polynomial import Polynomial
import pandas as pd
import math

from drift_optimizer import Drift_Optimizer
from bicycle import Bicycle

def compute_drift_dist(optimizer, visualize=False):
    plt.style.use('fivethirtyeight')
    data = pd.read_csv("C:/Users/conor/OneDrive/Documents/Projects/Fault Diagnostics/network_faults/data/noise_data.csv")
    dataset = data.to_numpy()

    dataset = np.reshape(dataset, (-1, 270, 4))
    drift_angles = []

    for i in range(dataset.shape[0]):
        color = 'black'
        xdata = dataset[i,:,0]
        y = dataset[i,:,1]
        label = dataset[i,0,-1]
        ydata = y

        popt, pcov = curve_fit(optimizer.func, xdata, ydata, bounds=([-math.radians(38)], [math.radians(38)]))
        print("Solution: ", *popt)

        drift_angles.append([*popt, label])

        if visualize:
            plt.scatter(xdata, ydata, color='#91bfdb', marker='.', label='healthy data')
            plt.ylabel('y (meters)')
            plt.xlabel('x (meters)')
            plt.legend()
            plt.title('Healthy Trajectory')

            plt.show()

            plt.scatter(xdata, ydata, color='#91bfdb', marker='.', label='healthy data')
            plt.scatter(optimizer.x_data, optimizer.func(xdata, *popt), color=color, marker='p')

            plt.ylabel('y (meters)')
            plt.xlabel('x (meters)')
            plt.legend()
            plt.title('Healthy Trajectory')
        
            plt.show()

    drift_angles = np.array(drift_angles)

    print("Dataset shape: ", dataset.shape)
    print("Drift shape: ", drift_angles.shape)
    print(drift_angles)

    healthy_drift = np.squeeze(drift_angles[np.argwhere(dataset[:,0,-1]==0)])
    left_drift = np.squeeze(drift_angles[np.argwhere(dataset[:,0,-1]==1), :])
    right_drift = np.squeeze(drift_angles[np.argwhere(dataset[:,0,-1]==2), :])
    right_severe_drift = np.squeeze(drift_angles[np.argwhere(dataset[:,0,-1]==3), :])

    print("Healthy drift shape: ", healthy_drift.shape)
    print(healthy_drift)

    plt.hist(healthy_drift[:, 0], density=True, bins=16)
    # plt.show()
    plt.hist(left_drift[:, 0], density=True, bins=16)
    # plt.show()
    plt.hist(right_drift[:, 0], density=True, bins=16)
    # plt.show()
    plt.hist(right_severe_drift[:, 0], density=True, bins=16)
    plt.show()

    healthy_mean, healthy_std = np.mean(healthy_drift[:, 0]), np.std(healthy_drift[:, 0])
    left_mean, left_std = np.mean(left_drift[:, 0]), np.std(left_drift[:, 0])
    right_mean, right_std = np.mean(right_drift[:, 0]), np.std(right_drift[:, 0])
    right_severe_mean, right_severe_std = np.mean(right_severe_drift[:, 0]), np.std(right_severe_drift[:, 0])

    drift_distributions = [[healthy_mean, healthy_std],
                           [left_mean, left_std],
                           [right_mean, right_std],
                           [right_severe_mean, right_severe_std]]

    return drift_distributions

def get_synthetic_data(distributions):
    for dist in distributions:
        drift_angle = np.random.normal(dist[0], dist[1])
        print("Synthetic Drift Angle: ", drift_angle)

        model = Bicycle(drift_angle)
        model.drive_along_path()

        ideal_model = Bicycle()
        ideal_model.drive_along_path()

        error_data = ideal_model.path_data - model.path_data

        # Plot results
        plt.scatter(np.arange(error_data.shape[0]), error_data[:, 0])
    plt.show()

if __name__ == "__main__":
    optimizer = Drift_Optimizer()

    distributions = compute_drift_dist(optimizer, False)

    closed_loop_data = get_synthetic_data(distributions)
