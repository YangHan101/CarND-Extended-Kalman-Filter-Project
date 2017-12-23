#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  std::cout << "x_before = " << x_ << std::endl;
	x_ = F_ * x_;
  std::cout << "x_after = " << x_ << std::endl;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    std::cout << "H = " << H_ << std::endl;
    Eigen::VectorXd y = z - H_ * x_;
    std::cout << "z = " << z << std::endl;
    std::cout << "y = " << y << std::endl;

    Eigen::MatrixXd Ht_ = H_.transpose();
    Eigen::MatrixXd S_ = H_ * P_ * Ht_ + R_;
    std::cout << "R = " << R_ << std::endl;
    std::cout << "P = " << P_ << std::endl;
    std::cout << "S = " << S_ << std::endl;
    Eigen::MatrixXd K_ = P_ * Ht_ * S_.inverse();
    std::cout << "K = " << K_ << std::endl;

    x_ = x_ + (K_ * y);
    std::cout << "x_updated = " << x_ << std::endl;
    Eigen::MatrixXd tmp_ = P_ - K_ * H_ * P_;
    P_ = tmp_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  Eigen::VectorXd z0 = Eigen::VectorXd(3);
  double r_ = std::sqrt(std::pow(x_(0), 2) + std::pow(x_(1), 2));
  double theta_ = std::atan2(x_(1), x_(0));
  double r_dot_ = (x_(0) * x_(2) + x_(1) * x_(3)) / r_;
  z0 << r_, theta_, r_dot_;

  Eigen::VectorXd y = z - z0;
  y(1) = fmod(y(1), 2 * M_PI);
  Eigen::MatrixXd S_ = H_ * P_ * H_.transpose() + R_;
  Eigen::MatrixXd K_ = P_ * H_.transpose() * S_.inverse();
  x_ = x_ + K_ * y;
  P_ = P_ - K_ * H_ * P_;

}
