import timeit

import numpy as np
from filterpy.kalman import KalmanFilter

state_dim = 4
measurement_dim = 2
kf = KalmanFilter(dim_x=state_dim, dim_z=measurement_dim)


kf.x = np.array([0, 0, 0, 0])
kf.P *= 1e-4
kf.F = np.array([[1, 0, 1, 0],
                 [0, 1, 0, 1],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])
kf.H = np.array([[1, 0, 0, 0],
                 [0, 1, 0, 0]])
kf.Q *= 1
kf.R *= 1

trajectory = [[2 * x, 3 * x] for x in range(120)]
def estimate():
    for z in trajectory:
        kf.predict()
        kf.update(np.array(z))
    for _ in range(10):
        pred = kf.predict()

print(timeit.timeit("estimate()", setup="from __main__ import estimate", number=100))