#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::max;

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
  x_ = F_ * x_;
  P_ = F_ * P_ * (F_.transpose()) + Q_;
}

void KalmanFilter::RegularUpdate(const VectorXd &y) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	RegularUpdate(z - (H_ * x_));
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  VectorXd z_pred(3);
  double rho = sqrt((x_(0)*x_(0)) + (x_(1)*x_(1)));
  z_pred << rho                                             ,
            atan2(x_(1),x_(0))                              ,
            ((x_(0)*x_(2))+(x_(1)*x_(3)))/ max(rho, 0.0001) ;

  VectorXd y = z - z_pred;

  while(y(1) > M_PI)
    y(1) -= 2*M_PI;

  while(y(1) < -M_PI)
    y(1) += 2*M_PI;
  
  RegularUpdate(y);
}
