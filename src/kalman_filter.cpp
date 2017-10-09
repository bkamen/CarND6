#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  x_ = F_*x_;
  MatrixXd F_t = F_.transpose();
  P_ = F_ * P_ * F_t + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y_ = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // new state
  x_ = x_ + (K*y_);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float ro = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float  theta = atan2(x_(1), x_(0));
  // checking division by zero
  float ro_dot;
  if (fabs(ro) < 0.0001) {
    ro_dot = 0;
  } else {
    ro_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / ro;
  }

  VectorXd z_pred(3);
  z_pred << ro, theta, ro_dot;

  VectorXd y_ = z - z_pred;
  y_(1) = atan2(sin(y_(1)),cos(y_(1)));
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // new state
  x_ = x_ + (K*y_);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}
