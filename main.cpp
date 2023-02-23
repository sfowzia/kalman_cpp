#include "kalman.h"

int main() {
  // Define the system dynamics and measurement models.
  MatrixXd A(2, 2);
  A << 1, 1, 0, 1;
  MatrixXd B(2, 1);
  B << 0, 1;
  MatrixXd C(1, 2);
  C << 1, 0;

  // Define the process and measurement noise covariances.
  MatrixXd Q(2, 2);
  Q << 1, 0, 0, 1;
  MatrixXd R(1, 1);
  R << 1;

  // Initialize the filter.
  KalmanFilter kf;
  kf.init(VectorXd::Zero(2), MatrixXd::Identity(2, 2));

  // Simulate the system.
  for (int i = 0; i < 10; i++) {
    // Simulate a control input and measurement.
    VectorXd u(1);
    u << i;
    VectorXd z(1);
    z << i;

    // Update the filter.
    kf.update(u, z, A, B, C, Q, R);

    // Print the current state estimate, innovation covariance, and innovation vector.
    std::cout << "Time step " << kf.getTimeStep() << ":" << std::endl;
    std::cout << "  State estimate: " << kf.getState().transpose() << std::endl;
    std::cout << "  Innovation covariance: " << kf.getInnovationCovariance() << std::endl;
    std::cout << "  Innovation vector: " << kf.getInnovation().transpose() << std::endl;
    std::cout << "  Innovation Mahalanobis distance: " << kf.getInnovationMahalanobisDistance() << std::endl;
    std::cout << "  Log-likelihood of measurement: " << kf.getLogLikelihood() << std::endl;
  }

  return 0;
}