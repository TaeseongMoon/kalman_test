#include "kalman_filter.h"

KalmanFilter::KalmanFilter(int state_dim, int measurement_dim)
    : state_dim_(state_dim), measurement_dim_(measurement_dim) {}

void KalmanFilter::initialize(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0,
                              const Eigen::MatrixXd& F, const Eigen::MatrixXd& H,
                              const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    x_ = x0;
    P_ = P0;
    F_ = F;
    H_ = H;
    Q_ = Q;
    R_ = R;
}

void KalmanFilter::predict() {
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd& z) {
    Eigen::VectorXd y = z - H_ * x_;
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = P_ - K * H_ * P_;
}

const Eigen::VectorXd& KalmanFilter::getState() const {
    return x_;
}

const Eigen::MatrixXd& KalmanFilter::getCovariance() const {
    return P_;
}