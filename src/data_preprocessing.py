import numpy as np
import pandas as pd
import math
import sys
from matplotlib import pyplot as plt
from scipy import stats
from scipy.optimize import curve_fit
from numpy.polynomial import Polynomial

def processData(path):
    #replace with the path on your pc
    data = pd.read_csv(path)
    dataset = data.to_numpy()

    run = []
    noise_functions = []
    last_iteration = 0.0
    label = 0
    count = 0
    healthy = []
    left = []
    right = []

    for i in range(dataset.shape[0]):
        if dataset[i, 1] < last_iteration or i == dataset.shape[0] - 1:
            run = np.array(run)
            run = np.reshape(run, (-1, 7))

            # TODO: clean run and calculate statistics
            eta = 0.1
            noise_function = cleanData(run, eta)
            noise_functions.append(np.concatenate((noise_function, label), axis=None))
            print(noise_function)

            if label == 0:
                healthy.append(run)
            elif label == 1:
                left.append(run)
            else:
                right.append(run)

            count += 1
            if count % 4 == 0:
                label += 1

            last_iteration = dataset[i, 1]
            run = []
            run.append(dataset[i, :])
        else:
            run.append(dataset[i, :])
            last_iteration = dataset[i, 1]

    healthy = np.array(healthy)
    print(healthy.shape)
    healthy = np.reshape(healthy, (healthy.shape[0]*healthy.shape[1], 7))

    left = np.array(left)
    print(left.shape)
    left = np.reshape(left, (-1, 7))

    right = np.array(right)
    print(right.shape)
    right = np.reshape(right, (right.shape[0]*right.shape[1], 7))

    f, (ax1, ax2, ax3) = plt.subplots(1, 3)
    color = 'black'
    xdata = healthy[:,2]
    print(xdata)
    y = healthy[:,3]
    ydata = y
    ax1.scatter(xdata, ydata, color='#91bfdb', marker='.')

    popt, pcov = curve_fit(func, xdata, ydata, bounds=([-20, -1.0, -100], [500, 10.0, 20]))
    print(popt)

    ax1.scatter(xdata, func(xdata, *popt), color=color, marker='p')

    ax1.set_ylabel('y')
    ax1.set_xlabel('x')
    ax1.set_xlim(-0.1,3.1)
    ax1.set_ylim(-0.6,0.95)
    ax1.legend()
    ax1.set_title('Healthy Trajectory')

    xdata = left[:,2]
    y = left[:,3]
    ydata = y
    ax2.scatter(xdata, ydata, color='#FFC39D', marker='.')

    popt, pcov = curve_fit(func, xdata, ydata, bounds=([-20, -1.0, -20], [20, 10.0, 20]))
    print(popt)

    ax2.scatter(xdata, func(xdata, *popt), color=color, marker='p')

    ax2.set_xlabel('x')
    ax2.set_xlim(-0.1,3.1)
    ax2.set_ylim(-0.6,0.95)
    ax2.legend()
    ax2.set_title('Left Fault Trajectory')

    xdata = right[:,2]
    y = right[:,3]
    ydata = y
    ax3.scatter(xdata, ydata, color='#99d594', marker='.')

    popt, pcov = curve_fit(func, xdata, ydata, method='trf', bounds=([-40, -1.0, -20], [20, 10.0, 20]))
    print(popt)

    ax3.scatter(xdata, func(xdata, *popt), color=color, marker='p')

    ax3.set_xlabel('x')
    ax3.set_xlim(-0.1,3.1)
    ax3.set_ylim(-0.6,0.95)
    ax3.legend()
    ax3.set_title('Right Fault Trajectory')

    plt.show()

    noise_functions = np.array(noise_functions)
    print(noise_functions)
    # healthy_distribution, left_distribution, right_distribution = computeDistribution(noise_functions)
    # distribution_data = np.array([healthy_distribution, left_distribution, right_distribution])
    # print(distribution_data.shape)
    f = open('/home/conor/catkin_ws/src/network_faults/data/noise_functions.csv', 'a')
    np.savetxt(f, noise_functions, delimiter=",")

def dynamics(x, y, theta, v, gamma, dt):
    yaw_dot = ((v/0.19)*(math.tan(gamma)))*dt
    x_dot = (v * math.cos(theta))*dt
    y_dot = (v * math.sin(theta))*dt

    theta = theta + yaw_dot
    x = x + x_dot
    y = y + y_dot

    return x, y, theta

def driveOpenLoop(gamma, max_iter):
    y_data = []
    i = 0
    x, y, theta = 0.0, 0.0, math.radians(0.0)

    while(i != max_iter):
        y_data.append(y)
        x, y, theta = dynamics(x, y, theta, 0.4, gamma, 0.1)
        i += 1

    return np.asarray(y_data)

def func(x, gamma):
    y = driveOpenLoop(gamma, x.shape[0])

    return y

def norm(x, x_hat):
    norm_vector = np.empty((x.shape[0], 1))
    for i in range(norm_vector.shape[0]):
        norm_vector[i] = np.linalg.norm(x[i, 3] - x_hat[i])

    return norm_vector

def zscoreOutliers(data, norm, iter):
    z = np.abs(stats.zscore(norm))
    data_out = data[(z < (3-iter*0.2)).all(axis=1)]

    return data_out

def iqrOutliers(data, norm, pct):
    q3, q1 = np.percentile(norm, [100-pct, pct])
    iqr = q3 - q1

    data_out = data[((norm > (q1 - 1.5*iqr)) | (norm < (q3 + 1.5*iqr))).all(axis=1)]
    # data_out = data_out[(norm > (q25 - 1.5*iqr)).all(axis=1)]

    return data_out

def cleanData(data, eta):
    print("New Run")
    run_norm = 100.0
    x = data
    iteration = 0
    max_iter = 1
    upper_bounds = np.array([math.radians(90)])
    lower_bounds = np.array([-math.radians(90)])
    range = 1.0
    count = 0.0
    run_opt, run_cov = curve_fit(func, x[:, 2], x[:, 3], bounds=(lower_bounds, upper_bounds))

    while run_norm >= eta and iteration != max_iter and x.shape[0] > 50:
        # upper_bounds = upper_bounds + upper_bounds*range
        # lower_bounds = lower_bounds + lower_bounds*range
        print("upper bounds")
        print(upper_bounds)
        print("lower bounds")
        print(lower_bounds)
        print("count: %s" % count)
        print("run norm: %s" % run_norm)
        popt, pcov = curve_fit(func, x[:, 2], x[:, 3], bounds=(lower_bounds, upper_bounds))
        y_hat = func(x[:, 2], *popt)
        plt.scatter(x[:, 2], x[:, 3], color='red', marker='p')
        plt.scatter(x[:, 2], y_hat, color='blue', marker='p')

        norm_vector = norm(x, y_hat)

        data_zscore = zscoreOutliers(x, norm_vector, iteration)
        data_iqr = iqrOutliers(x, norm_vector, 25)

        zscore_popt, zscore_pcov = curve_fit(func, data_zscore[:, 2], data_zscore[:, 3], bounds=(lower_bounds, upper_bounds))
        zscore_y_hat = func(data_zscore[:, 2], *zscore_popt)

        iqr_popt, iqr_pcov = curve_fit(func, data_iqr[:, 2], data_iqr[:, 3], bounds=(lower_bounds, upper_bounds))
        iqr_y_hat = func(data_iqr[:, 2], *iqr_popt)

        zscore_norm_vector = norm(data_zscore, zscore_y_hat)
        iqr_norm_vector = norm(data_iqr, iqr_y_hat)

        zscore_norm = np.linalg.norm(data_zscore[:, 3] - zscore_y_hat)
        iqr_norm = np.linalg.norm(data_iqr[:, 3] - iqr_y_hat)
        print("Zscore norm: %s" % zscore_norm)
        print("IQR norm: %s" % iqr_norm)

        print(x.shape)
        print(data_zscore.shape)
        print(data_iqr.shape)
        print(zscore_y_hat.shape)
        print(iqr_y_hat.shape)

        if zscore_norm < iqr_norm:
            plt.scatter(data_zscore[:, 2], data_zscore[:, 3], color='red', label="sample data: iteration %s" % iteration)
            plt.scatter(data_zscore[:, 2], zscore_y_hat, color='blue', label="fitted data: iteration %s" % iteration)
            plt.title("Zscore Filtered Data")
            plt.legend()

            run_opt = zscore_popt
            run_norm = zscore_norm
            x = data_zscore
            iteration += 1
        else:
            plt.scatter(data_iqr[:, 2], data_iqr[:, 3], color='red', label="sample data: iteration %s" % iteration)
            plt.scatter(data_iqr[:, 2], iqr_y_hat, color='blue', label="fitted data: iteration %s" % iteration)
            plt.title("IQR Filtered Data")
            plt.legend()

            run_opt = iqr_popt
            run_norm = iqr_norm
            x = data_iqr
            iteration += 1

        range = 1.0
        count += 1

        plt.show()

    return run_opt

def computeDistribution(x):
    healthy_data = x[(x[:, -1] == 0)]
    left_data = x[(x[:, -1] == 1)]
    right_data = x[(x[:, -1] == 2)]

    print(np.std(healthy_data[:, 0], dtype=np.float64))

    healthy_distribution = np.concatenate((np.mean(healthy_data[:, 0]), np.mean(healthy_data[:, 1]), np.mean(healthy_data[:, 2]),
                                    np.std(healthy_data[:, 0]), np.std(healthy_data[:, 1]), np.std(healthy_data[:, 2]), healthy_data[0, -1]), axis=None)
    left_distribution = np.concatenate((np.mean(left_data[:, 0]), np.mean(left_data[:, 1]), np.mean(left_data[:, 2]),
                                    np.std(left_data[:, 0]), np.std(left_data[:, 1]), np.std(left_data[:, 2]), left_data[0, -1]), axis=None)
    right_distribution = np.concatenate((np.mean(right_data[:, 0]), np.mean(right_data[:, 1]), np.mean(right_data[:, 2]),
                                    np.std(right_data[:, 0]), np.std(right_data[:, 1]), np.std(right_data[:, 2]), right_data[0, -1]), axis=None)
    print("Healthy Distribution")
    print(healthy_distribution)
    print("Left Distribution")
    print(left_distribution)
    print("Right Distribution")
    print(right_distribution)

    return healthy_distribution, left_distribution, right_distribution

if __name__ == '__main__':
    path = "C:/Users/conor/OneDrive/Documents/Projects/Fault Diagnostics/network_faults/data/noise_data.csv"
    processData(path)
