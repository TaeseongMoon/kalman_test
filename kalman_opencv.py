import timeit

import cv2
import numpy as np


class KalmanFilter:
    def __init__(self, state_noise=0.02, sensor_noise=2):
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array(
            [[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32
        )
        self.kf.processNoiseCov = (
            np.array(
                [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32
            )
            * state_noise
        )
        self.kf.measurementNoiseCov = (
            np.array([[1, 0], [0, 1]], np.float32) * sensor_noise
        )
        self.trajectory = [[2 * x, 3 * x] for x in range(120)]

    def init_kf(self, trajectory):
        trajectory = np.expand_dims(trajectory.astype(np.float32), axis=2)

        for traj in trajectory:
            self.kf.predict()
            self.kf.correct(traj)

    def update_kf(self, coord):
        predict = self.kf.predict()[:2]
        self.kf.correct(np.array(coord, dtype=np.float32))
        return [int(predict[0]), int(predict[1])], self.kf.errorCovPost[0][0]

    def estimate(self, times=10):
        # predicted_coords = []
        # prediction_errors = []
        self.init_kf(np.array(self.trajectory))
        for _ in range(times):
            self.kf.predict()
            # predicted_coords.append(predicted)
            # prediction_errors.append(self.kf.errorCovPost[0][0])

        return np.array(predicted_coords, int).squeeze().tolist(), prediction_errors


kf = KalmanFilter()


def kalman_iter():
    kf.estimate()


print(
    timeit.timeit(
        "kalman_iter()",
        setup="from __main__ import kalman_iter",
        number=100,
    )
)
