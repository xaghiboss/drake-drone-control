#pragma once

#include "drake/systems/framework/leaf_system.h"
#include <Eigen/Dense>

namespace drake {
namespace systems {

// ============================================================================
// EKF State Estimator - Fuses IMU measurements for optimal state estimation
// ============================================================================
// 
// State vector (13 dimensions):
//   [0-3]   Quaternion (w, x, y, z) - orientation
//   [4-6]   Position (x, y, z) - world frame
//   [7-9]   Velocity (vx, vy, vz) - world frame
//   [10-12] Gyro bias (bx, by, bz) - accumulated drift
//
// Measurements (7 dimensions):
//   [0-2]   Accelerometer (ax, ay, az) - body frame, includes gravity
//   [3-5]   Gyroscope (wx, wy, wz) - body frame
//   [6]     Barometer altitude (z) - world frame
//
class EkfEstimator : public LeafSystem<double> {
 public:
  EkfEstimator(double update_rate = 200.0);

 private:
  // Discrete update (EKF prediction + correction)
  void UpdateEstimate(const Context<double>& context,
                      DiscreteValues<double>* discrete_state) const;
  
  // Output current state estimate
  void CalcControllerOutput(const Context<double>& context,
                            BasicVector<double>* output) const;
  
  // EKF prediction step (process model)
  void Predict(Eigen::VectorXd& x, Eigen::MatrixXd& P, double dt, const Eigen::Vector3d& gyro_meas) const;
  
  // EKF correction step (measurement update)
  void Correct(Eigen::VectorXd& x, Eigen::MatrixXd& P, 
               const Eigen::VectorXd& z) const;
  
  // Process model: x_new = f(x_old, u, dt)
  Eigen::VectorXd ProcessModel(const Eigen::VectorXd& x, 
                                const Eigen::VectorXd& u, 
                                double dt) const;
  
  // Measurement model: z = h(x)
  Eigen::VectorXd MeasurementModel(const Eigen::VectorXd& x) const;
  
  // Jacobian of process model
  Eigen::MatrixXd ProcessJacobian(const Eigen::VectorXd& x, double dt) const;
  
  // Jacobian of measurement model
  Eigen::MatrixXd MeasurementJacobian(const Eigen::VectorXd& x) const;
  
  // Normalize quaternion in state vector
  void NormalizeQuaternion(Eigen::VectorXd& x) const;
  
  // Constants
  const double update_period_;
  const double gravity_ = 9.81;
  
  // Process noise covariance Q (tunable!)
  Eigen::MatrixXd Q_;  // 13x13
  
  // Measurement noise covariance R (from sensor specs)
  Eigen::MatrixXd R_;  // 7x7
  
  // Initial state covariance
  Eigen::MatrixXd P0_; // 13x13
  
  // State dimension
  static constexpr int STATE_DIM = 13;
  static constexpr int MEAS_DIM = 7;
};

}  // namespace systems
}  // namespace drake