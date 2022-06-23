import time

import numpy as np
from scipy.optimize import minimize as m
import moveit_commander
import sys

moveit_commander.roscpp_initialize(sys.argv)
group = moveit_commander.MoveGroupCommander("saga_arm")

target = [0.1, 0.25, 1.1]  # in base_link

def opt_fun1(qs):
    q1, q2, q3 = qs
    return q1**2 + q2**2 + q3**2

def jac1(qs):
    q1, q2, q3 = qs
    return np.array([2*q1, 2*q2, 2*q3])

def opt_fun2(qs):
    q1, q2, q3 = qs
    x, y = fwd_kin(qs)
    return (target[0] - x)**2 + (target[1] - y)**2 + 0.0001 * q1**2 + 0.0001 * q2**2 + 0.0001 * q3**2

def fwd_kin(qs, coeff):
    q1, q2, q3 = qs
    a1, a2, a3 = coeff[0], coeff[1], coeff[2]
    x = a1 * np.cos(q1) + a1 * np.cos(q1 - q2) + a2 * np.cos(q1 - q2 + q3) + a3
    y = a1 * np.sin(q1) + a1 * np.sin(q1 - q2) + a2 * np.sin(q1 - q2 + q3)
    return x, y

def jac2(qs):
    q1, q2, q3 = qs
    dxdq1 = 0.2 * np.sin(q1) + 0.2 * np.sin(q1 - q2) + 0.24 * np.sin(q1 - q2 + q3)
    dxdq2 = - 0.2 * np.sin(q1 - q2) - 0.24 * np.sin(q1 - q2 + q3)
    dxdq3 = 0.24 * np.sin(q1 - q2 + q3)

    dydq1 = -0.2 * np.cos(q1) - 0.2 * np.cos(q1 - q2) - 0.24 * np.cos(q1 - q2 + q3)
    dydq2 = 0.2 * np.cos(q1 - q2) + 0.24 * np.cos(q1 - q2 + q3)
    dydq3 = -0.24 * np.cos(q1 - q2 + q3)

    x, y = fwd_kin(qs)
    dFdq1 = 2 * (x - target[0]) * dxdq1 + 2 * (y - target[1]) * dydq1  # + 2*q1
    dFdq2 = 2 * (x - target[0]) * dxdq2 + 2 * (y - target[1]) * dydq2  # + 2*q2
    dFdq3 = 2 * (x - target[0]) * dxdq3 + 2 * (y - target[1]) * dydq3  # + 2*q3
    return np.array([dFdq1, dFdq2, dFdq3])

def constraint_fun(qs):
    xt, yt, zt = target
    xf, yf = fwd_kin(qs)
    return np.array([xf - xt, yf - yt])

constraints = {'type': 'eq',
                'fun': constraint_fun
               }
pi = np.pi
bounds = ((-pi/2, pi/2), (-pi/2, pi/2), (-pi/2, pi/2))
init_guess = group.get_current_joint_values()[1:4]

# init_guess = np.random.uniform(-pi/2, pi/2, 3)

result1 = m(opt_fun1, init_guess, method='SLSQP', constraints=constraints, tol=1e-9, jac=jac1,
           options={'ftol': 1e-9, 'maxiter': 1000, 'disp': True}, bounds=bounds)

print "########"
result2 = m(opt_fun2, init_guess, method='SLSQP', tol=1e-9, jac=jac2,
           options={'ftol': 1e-9, 'maxiter': 1000, 'disp': True}, bounds=bounds)

