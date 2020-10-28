import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from pid import PID
from bicycle import Bicycle

def main(model):
    model.driveOpenLoop(0, 'blue')


if __name__ == '__main__':
    target = np.array([10.0, 10.0])
    bicycle = Bicycle(target)
    main(bicycle)
