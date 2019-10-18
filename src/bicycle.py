import numpy as np
import math
import matplotlib.pyplot as plt

test_theta = 0.0
test_x = 0.0
test_y = 0.0
path = []

def model(v, gamma):
    global test_theta, test_x, test_y, path
    L = 0.19

    #dynamics
    theta_dot = ((v/L)*(math.tan(gamma)))
    x_dot = v * math.cos(test_theta)
    y_dot = v * math.sin(test_theta)

    #derivatives
    test_theta= test_theta + theta_dot
    test_x = test_x + x_dot
    test_y = test_y + y_dot
    path.append([test_x, test_y])


desired_x = 1
desired_y = 1
delta_x = desired_x - test_x
delta_y = desired_y - test_y
Kv = 0.005
Kh = 1

while(math.sqrt(delta_x**2 + delta_y**2) > 0.2):
    delta_x = desired_x - test_x
    delta_y = desired_y - test_y
    theta = math.degrees(math.atan2(delta_y, delta_x))
    delta_theta = theta - test_theta

    distance = math.sqrt(delta_x**2 + delta_y**2)
    print("distance: %s" % distance)
    model(distance*Kv, delta_theta*Kh)
path = np.asarray(path)
plt.plot(path[:, 0], path[:, 1])
plt.show()
