#ifndef KALMAN_H
#define KALMAN_H

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

class KalmanFilter {
 public:
  KalmanFilter() {}

  // Initializes the filter with the specified initial state and covariance.
  void init(VectorXd x0, MatrixXd P0) {
    x_ = x0;
    P_ = P0;
  }

  // Runs one iteration of the Kalman filter given a control input and measurement.
  void update(VectorXd u, VectorXd z, MatrixXd A, MatrixXd B,
              MatrixXd C, MatrixXd Q, MatrixXd R) {
    // Predict the next state estimate using the system dynamics.
    x_ = A * x_ + B * u;
    P_ = A * P_ * A.transpose() + Q;

    // Compute the Kalman gain.
    MatrixXd S = C * P_ * C.transpose() + R;
    MatrixXd K = P_ * C.transpose() * S.inverse();

    // Update the state estimate based on the measurement.
    x_ = x_ + K * (z - C * x_);
    P_ = (MatrixXd::Identity(P_.rows(), P_.cols()) - K * C) * P_;

    // Update the innovation covariance and innovation vector.
    innovation_covariance_ = C * P_ * C.transpose() + R;
    innovation_ = z - C * x_;

    // Compute the innovation Mahalanobis distance.
    innovation_mahalanobis_distance_ = std::sqrt(innovation_.transpose()
        * innovation_covariance_.inverse() * innovation_);

    // Update the log-likelihood of the measurement.
    log_likelihood_ = -0.5 * (std::log(2 * M_PI) + std::log(innovation_covariance_.determinant())
        + innovation_.transpose()* innovation_covariance_.inverse() * innovation_);

    // Increment the time step.
    time_step_++;
  }

  // Returns the current state estimate.
  VectorXd getState() const {
    return x_;
  }

  // Returns the current innovation covariance.
  MatrixXd getInnovationCovariance() const {
    return innovation_covariance_;
  }

  // Returns the current innovation vector.
  VectorXd getInnovation() const {
    return innovation_;
  }

  // Returns the current innovation Mahalanobis distance.
  double getInnovationMahalanobisDistance() const {
    return innovation_mahalanobis_distance_;
  }

  // Returns the log-likelihood of the most recent measurement.
  double getLogLikelihood() const {
    return log_likelihood_;
  }

  // Returns the current time step.
  int getTimeStep() const {
    return time_step_;
  }

 private:
  VectorXd x_; // State estimate.
  MatrixXd P_; // State covariance.
  MatrixXd innovation_covariance_; // Innovation covariance.
  VectorXd innovation_; // Innovation vector.
  double innovation_mahalanobis_distance_; // Innovation Mahalanobis distance.
  double log_likelihood_; // Log-likelihood of the most recent measurement.
  int time_step_ = 0; // Current time step.
};

#endif