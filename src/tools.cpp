#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  Eigen::VectorXd sum;
  sum = VectorXd(4);
  sum << 0, 0, 0, 0;
  for (int i=0; i < estimations.size(); i++){
    Eigen::VectorXd diff = estimations[i] - ground_truth[i];
    sum += diff.cwiseProduct(diff);

  }
  return (sum / estimations.size()).array().sqrt();

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  double x_ = x_state(0);
  double y_ = x_state(1);
  double vx_ = x_state(2);
  double vy_ = x_state(3);

  Eigen::MatrixXd Hj_;

  Hj_ = Eigen::MatrixXd(3, 4);
  double r2_ = x_ * x_ + y_ * y_;
  double r_ = std::sqrt(r2_);

  if (r_ > 0.01) {
    Hj_ << x_ / r_, y_ / r_, 0, 0,
           -y_ / r2_, x_ / r2_, 0, 0,
           y_ * (vx_ * y_ - vy_ * x_) / (r2_ * r_), x_ * (vy_ * x_ - vx_ * y_) / (r2_ * r_), x_ / r_, y_ / r_;
  }
  else {
    cout << "divided by zero in CalculateJacobian" << endl;
  }

  return Hj_;
}
