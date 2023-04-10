#pragma once

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter(int state_dim, int measurement_dim);

    void initialize(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0,
                    const Eigen::MatrixXd& F, const Eigen::MatrixXd& H,
                    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);

    void predict();
    void update(const Eigen::VectorXd& z);

    const Eigen::VectorXd& getState() const;
    const Eigen::MatrixXd& getCovariance() const;

private:
    int state_dim_;
    int measurement_dim_;

    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;

    Eigen::MatrixXd F_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
};