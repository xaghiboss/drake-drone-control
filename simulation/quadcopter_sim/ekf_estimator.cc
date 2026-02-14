#include "ekf_estimator.h"
#include <cmath>

namespace drake {
namespace systems {

EkfEstimator::EkfEstimator(double update_rate)
    : update_period_(1.0 / update_rate) {
  
  // Input port: IMU raw measurements (7 elements)
  this->DeclareVectorInputPort("imu_measurements", MEAS_DIM);
  
  // Output port: State estimate in CONTROLLER FORMAT (13 elements)
  this->DeclareVectorOutputPort(
      "controller_state", 
      13,
      [this](const Context<double>& context, BasicVector<double>* output) {
        this->CalcControllerOutput(context, output);
      });
  
  // Discrete state: [state(13), covariance(13x13 flattened), initialized_flag(1)]
  const int discrete_size = STATE_DIM + STATE_DIM * STATE_DIM + 1;
  this->DeclareDiscreteState(discrete_size);
  
  // Periodic update at EKF rate
  this->DeclarePeriodicDiscreteUpdateEvent(
      update_period_, 0.0,
      &EkfEstimator::UpdateEstimate);
  
  // ========================================================================
  // INITIALIZE NOISE COVARIANCES
  // ========================================================================
  
  Q_ = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
  
  // Quaternion process noise (gyro integration drift)
  Q_(0, 0) = 1e-6;
  Q_(1, 1) = 1e-6;
  Q_(2, 2) = 1e-6;
  Q_(3, 3) = 1e-6;
  
  // Position process noise
  Q_(4, 4) = 1e-4;  // x
  Q_(5, 5) = 1e-4;  // y
  Q_(6, 6) = 1e-4;  // z
  
  // Velocity process noise
  Q_(7, 7) = 0.01;  // vx
  Q_(8, 8) = 0.01;  // vy
  Q_(9, 9) = 0.01;  // vz
  
  // Gyro bias process noise
  Q_(10, 10) = 1e-8;
  Q_(11, 11) = 1e-8;
  Q_(12, 12) = 1e-8;
  
  // Measurement noise R
  R_ = Eigen::MatrixXd::Zero(MEAS_DIM, MEAS_DIM);
  
  // Accelerometer noise
  R_(0, 0) = 0.02 * 0.02;
  R_(1, 1) = 0.02 * 0.02;
  R_(2, 2) = 0.02 * 0.02;
  
  // Gyroscope noise
  R_(3, 3) = 0.01 * 0.01;
  R_(4, 4) = 0.01 * 0.01;
  R_(5, 5) = 0.01 * 0.01;
  
  // Barometer noise
  R_(6, 6) = 0.05 * 0.05;
  
  // Initial state covariance
  P0_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  P0_(0, 0) = 0.1;
  P0_(1, 1) = 0.1;
  P0_(2, 2) = 0.1;
  P0_(3, 3) = 0.1;
  P0_(4, 4) = 1.0;
  P0_(5, 5) = 1.0;
  P0_(6, 6) = 1.0;
  P0_(7, 7) = 1.0;
  P0_(8, 8) = 1.0;
  P0_(9, 9) = 1.0;
  P0_(10, 10) = 0.001;
  P0_(11, 11) = 0.001;
  P0_(12, 12) = 0.001;
}

void EkfEstimator::UpdateEstimate(const Context<double>& context,
                                   DiscreteValues<double>* discrete_state) const {
  
  // Get current discrete state
  Eigen::VectorXd state_vec = discrete_state->get_vector(0).CopyToVector();
  
  // Extract components
  Eigen::VectorXd x = state_vec.head(STATE_DIM);
  Eigen::Map<Eigen::MatrixXd> P_flat(state_vec.data() + STATE_DIM, STATE_DIM, STATE_DIM);
  Eigen::MatrixXd P = P_flat;
  double initialized_flag = state_vec(STATE_DIM + STATE_DIM * STATE_DIM);
  
  // Get IMU measurement
  const auto& z = this->get_input_port(0).Eval(context);
  
  // Check if first iteration
  if (initialized_flag < 0.5) {
    // INITIALIZATION: Set state from first measurement
    
    // Start with identity quaternion (upright)
    x(0) = 1.0;  // w
    x(1) = 0.0;  // x
    x(2) = 0.0;  // y
    x(3) = 0.0;  // z
    
    // Position: Set Z from barometer, X/Y unknown (start at origin)
    x(4) = 0.0;
    x(5) = 0.0;
    x(6) = 0.0;  // ← FIXED: Start at ground, not from noisy barometer!
    
    // Velocity: zero
    x(7) = 0.0;
    x(8) = 0.0;
    x(9) = 0.0;
    
    // Gyro bias: zero
    x(10) = 0.0;
    x(11) = 0.0;
    x(12) = 0.0;
    
    P = P0_;
    initialized_flag = 1.0;
    
  } else {
    // NORMAL OPERATION: Predict + Correct
    
    // Extract gyro from measurement for prediction
    Eigen::Vector3d gyro_meas = z.segment(3, 3);
    
    // STEP 1: Prediction
    Predict(x, P, update_period_, gyro_meas);  // ← FIXED: Pass gyro!
    
    // STEP 2: Correction
    Correct(x, P, z);
  }
  
  // Normalize quaternion
  NormalizeQuaternion(x);
  
  // Pack back
  state_vec.head(STATE_DIM) = x;
  Eigen::Map<Eigen::VectorXd> P_vec(P.data(), STATE_DIM * STATE_DIM);
  state_vec.segment(STATE_DIM, STATE_DIM * STATE_DIM) = P_vec;
  state_vec(STATE_DIM + STATE_DIM * STATE_DIM) = initialized_flag;
  
  discrete_state->get_mutable_vector(0).SetFromVector(state_vec);
}

void EkfEstimator::Predict(Eigen::VectorXd& x, Eigen::MatrixXd& P, double dt,
                            const Eigen::Vector3d& gyro_meas) const {
  // Extract state components
  Eigen::Quaterniond q(x(0), x(1), x(2), x(3));
  Eigen::Vector3d pos = x.segment(4, 3);
  Eigen::Vector3d vel = x.segment(7, 3);
  Eigen::Vector3d gyro_bias = x.segment(10, 3);
  
  // CRITICAL FIX: Use actual gyro measurement, corrected by bias estimate
  Eigen::Vector3d omega = gyro_meas - gyro_bias;
  
  // Predict orientation (quaternion integration)
  // Using simplified Euler integration: q_new = q_old + 0.5 * q_old * omega * dt
  Eigen::Quaterniond omega_quat(0, omega(0), omega(1), omega(2));
  Eigen::Quaterniond q_dot;
  q_dot.w() = -0.5 * (q.x() * omega(0) + q.y() * omega(1) + q.z() * omega(2));
  q_dot.x() =  0.5 * (q.w() * omega(0) + q.y() * omega(2) - q.z() * omega(1));
  q_dot.y() =  0.5 * (q.w() * omega(1) + q.z() * omega(0) - q.x() * omega(2));
  q_dot.z() =  0.5 * (q.w() * omega(2) + q.x() * omega(1) - q.y() * omega(0));
  
  q.w() += q_dot.w() * dt;
  q.x() += q_dot.x() * dt;
  q.y() += q_dot.y() * dt;
  q.z() += q_dot.z() * dt;
  q.normalize();
  
  // Predict position (velocity integration)
  pos += vel * dt;
  
  // Predict velocity (gravity only - no thrust in prediction)
  Eigen::Vector3d gravity_world(0, 0, -gravity_);
  vel += gravity_world * dt;
  
  // Gyro bias stays constant (random walk)
  
  // Pack back into state
  x(0) = q.w();
  x(1) = q.x();
  x(2) = q.y();
  x(3) = q.z();
  x.segment(4, 3) = pos;
  x.segment(7, 3) = vel;
  x.segment(10, 3) = gyro_bias;
  
  // Compute process Jacobian F
  Eigen::MatrixXd F = ProcessJacobian(x, dt);
  
  // Predict covariance: P = F * P * F^T + Q
  P = F * P * F.transpose() + Q_;
}

void EkfEstimator::Correct(Eigen::VectorXd& x, Eigen::MatrixXd& P, 
                            const Eigen::VectorXd& z) const {
  
  // Predicted measurement: z_pred = h(x)
  Eigen::VectorXd z_pred = MeasurementModel(x);
  
  // Innovation (measurement residual)
  Eigen::VectorXd y = z - z_pred;
  
  // Measurement Jacobian H
  Eigen::MatrixXd H = MeasurementJacobian(x);
  
  // Innovation covariance: S = H * P * H^T + R
  Eigen::MatrixXd S = H * P * H.transpose() + R_;
  
  // Kalman gain: K = P * H^T * S^(-1)
  Eigen::MatrixXd K = P * H.transpose() * S.inverse();
  
  // Update state: x = x + K * y
  x += K * y;
  
  // Update covariance: P = (I - K * H) * P
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  P = (I - K * H) * P;
}

Eigen::VectorXd EkfEstimator::MeasurementModel(const Eigen::VectorXd& x) const {
  // Extract state
  Eigen::Quaterniond q(x(0), x(1), x(2), x(3));
  Eigen::Vector3d pos = x.segment(4, 3);
  
  Eigen::VectorXd z_pred(MEAS_DIM);
  
  // CRITICAL FIX: Accelerometer measures gravity in body frame
  // When drone is level: body Z-axis aligns with world Z
  // When tilted: gravity vector rotates in body frame
  Eigen::Vector3d gravity_world(0, 0, gravity_);
  Eigen::Vector3d gravity_body = q.inverse() * gravity_world;
  
  z_pred.head(3) = gravity_body;  // Predicted accelerometer reading
  
  // Gyroscope: Predict zero (bias is already subtracted in prediction)
  z_pred.segment(3, 3) = Eigen::Vector3d::Zero();
  
  // Barometer: Measures altitude directly
  z_pred(6) = pos(2);
  
  return z_pred;
}

Eigen::MatrixXd EkfEstimator::ProcessJacobian(const Eigen::VectorXd& x, double dt) const {
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
  
  // Position depends on velocity
  F(4, 7) = dt;
  F(5, 8) = dt;
  F(6, 9) = dt;
  
  // Simplified: Full Jacobian would include quaternion-velocity coupling
  
  return F;
}

Eigen::MatrixXd EkfEstimator::MeasurementJacobian(const Eigen::VectorXd& x) const {
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(MEAS_DIM, STATE_DIM);
  
  // CRITICAL FIX: Compute Jacobian of accelerometer w.r.t quaternion
  // This tells the EKF: "accelerometer readings depend on orientation"
  Eigen::Quaterniond q(x(0), x(1), x(2), x(3));
  Eigen::Vector3d gravity_world(0, 0, gravity_);
  
  // Simplified Jacobian (full version requires quaternion derivative math)
  // For small angles: accel depends linearly on quaternion components
  // ∂(q^-1 * g)/∂q ≈ sensitivity matrix
  
  // Accel[0] (x-axis) depends on quaternion y and z (roll/pitch)
  H(0, 2) = -2.0 * gravity_;  // qy affects accel_x
  H(0, 3) = 2.0 * gravity_;   // qz affects accel_x
  
  // Accel[1] (y-axis) depends on quaternion x and z
  H(1, 1) = 2.0 * gravity_;   // qx affects accel_y
  H(1, 3) = -2.0 * gravity_;  // qz affects accel_y
  
  // Accel[2] (z-axis) depends on quaternion w and z
  H(2, 0) = 2.0 * gravity_;   // qw affects accel_z
  H(2, 2) = -2.0 * gravity_;  // qy affects accel_z
  
  // Gyro measurements depend on gyro bias (critical for drift correction!)
  H(3, 10) = -1.0;  // gyro_x measurement affected by bias_x
  H(4, 11) = -1.0;  // gyro_y measurement affected by bias_y
  H(5, 12) = -1.0;  // gyro_z measurement affected by bias_z
  
  // Barometer measures altitude (position z) directly
  H(6, 6) = 1.0;
  
  return H;
}

void EkfEstimator::NormalizeQuaternion(Eigen::VectorXd& x) const {
  Eigen::Quaterniond q(x(0), x(1), x(2), x(3));
  q.normalize();
  x(0) = q.w();
  x(1) = q.x();
  x(2) = q.y();
  x(3) = q.z();
}

void EkfEstimator::CalcControllerOutput(const Context<double>& context,
                                         BasicVector<double>* output) const {
  // Get EKF state estimate
  const Eigen::VectorXd state_vec = context.get_discrete_state(0).CopyToVector();
  const Eigen::VectorXd x = state_vec.head(STATE_DIM);
  
  // Extract components
  Eigen::Quaterniond q(x(0), x(1), x(2), x(3));
  Eigen::Vector3d position = x.segment(4, 3);
  Eigen::Vector3d velocity = x.segment(7, 3);
  Eigen::Vector3d gyro_bias = x.segment(10, 3);
  
  // Get current IMU measurement
  const auto& imu = this->get_input_port(0).Eval(context);
  Eigen::Vector3d accel_meas = imu.head(3);
  Eigen::Vector3d gyro_meas = imu.segment(3, 3);
  
  // Correct gyro with bias estimate
  Eigen::Vector3d angular_vel = gyro_meas - gyro_bias;
  
  // Pack into controller format: [quat(4), angular_vel(3), accel(3), pos(3)]
  Eigen::VectorXd controller_state(13);
  controller_state << q.w(), q.x(), q.y(), q.z(),
                      angular_vel,
                      accel_meas,
                      position;
  
  output->SetFromVector(controller_state);
}

}  // namespace systems
}  // namespace drake