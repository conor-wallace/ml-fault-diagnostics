import numpy as np
import scipy
import sympy
from sympy.abc import alpha, x, y, v, w, R, theta
from sympy import symbols, Matrix

# class EKF():
#     def __init__(self, n, m, pval=0.1, qval=1e-4, rval=0.1):
#         '''
#         Creates a KF object with n states, m observables, and specified values for
#         prediction noise covariance pval, process noise covariance qval, and
#         measurement noise covariance rval.
#         '''
#
#         # No previous prediction noise covariance
#         self.P_ = None
#
#         # Current state is zero, with diagonal noise covariance matrix
#         self.x = np.zeros(n)
#         self.P_post = np.eye(n) * pval
#
#         # Set up covariance matrices for process noise and measurement noise
#         self.Q = np.eye(n) * qval
#         self.R = np.eye(m) * rval
#
#         # Identity matrix will be usefel later
#         self.I = np.eye(n)

# sympy.init_printing(use_la)
time = symbols('t')
d = v*time
beta = (d/w)*sympy.tan(alpha)
r = w/sympy.tan(alpha)
L = 0.19

fxu = Matrix([[x + v*sympy.cos(theta)],
              [y + v*sympy.sin(theta)],
              [theta + (v*sympy.tan(alpha))/L]])

F = fxu.jacobian(Matrix([x, y, theta]))
print(F)

V = fxu.jacobian(Matrix([v, alpha]))
print(V)
