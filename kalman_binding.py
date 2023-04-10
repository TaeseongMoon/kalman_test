import timeit

import numpy as np

import kalman_filter

state_dim = 4
measurement_dim = 2
kf = kalman_filter.KalmanFilter(state_dim, measurement_dim)

# 초기 상태, 공분산 행렬, 시스템 행렬, 측정 행렬, 과정 및 측정 노이즈 행렬 설정
x0 = np.array([0, 0, 0, 0])
P0 = np.eye(state_dim) * 1e-4
F = np.array([[1, 0, 1, 0],
              [0, 1, 0, 1],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])
H = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0]])
Q = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])
R = np.eye(measurement_dim)

kf.initialize(x0, P0, F, H, Q, R)


trajectory = [[2 * x, 3 * x] for x in range(120)]

def estimate():
    for z in trajectory:
        kf.predict()
        kf.update(np.array(z))
    for _ in range(10):
        pred = kf.predict()



print(timeit.timeit("estimate()", setup="from __main__ import estimate", number=100))