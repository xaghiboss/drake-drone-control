#include "imu_sensor.h"
#include <random>
#include "drake/math/rotation_matrix.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace systems {

ImuSensor::ImuSensor(const multibody::MultibodyPlant<double>* plant,
                     const multibody::RigidBody<double>* body,
                     double update_rate)
    : plant_(plant), body_(body) {
  
  // Input: plant state (full state vector)
  this->DeclareVectorInputPort("plant_state",
                               plant->num_positions() + plant->num_velocities());
  
  // Output: IMU measurement vector (13 elements to match controller input!)
  // [0-3]   Quaternion (w, x, y, z)
  // [4-6]   Angular velocity (body frame, rad/s)
  // [7-9]   Linear acceleration (body frame, m/s²)
  // [10-12] Position (world frame, m)
  this->DeclareVectorOutputPort("imu_measurement", 7,
                                &ImuSensor::CalcImuOutput);
  
  // Discrete state: stores last IMU reading (for fixed-rate output)
  this->DeclareDiscreteState(7);
  
  // Periodic update at IMU rate (e.g., 200Hz)
  const double period = 1.0 / update_rate;
  this->DeclarePeriodicDiscreteUpdateEvent(
      period, 0.0,  // period, offset
      &ImuSensor::UpdateImuState);
}

void ImuSensor::UpdateImuState(const Context<double>& context,
                                DiscreteValues<double>* discrete_state) const {
  // Read plant state from input port
  const auto& x = this->get_input_port(0).Eval(context);
  const int nq = plant_->num_positions();
  const int nv = plant_->num_velocities();
  
  Eigen::VectorXd q = x.head(nq);
  Eigen::VectorXd v = x.tail(nv);
  
  // ========================================================================
  // EXTRACT STATE DIRECTLY FROM VECTORS - PERFECT (NO NOISE!)
  // ========================================================================
  
  // 1. ORIENTATION (Quaternion) - directly from q
  Eigen::Quaterniond quat(q(0), q(1), q(2), q(3));
  quat.normalize();
  
  // REMOVED: Orientation noise (perfect quaternion!)
  
  // 2. ANGULAR VELOCITY (Gyroscope) - directly from v
  // For a floating body, v = [angular_velocity(3), linear_velocity(3)]
  const Eigen::Vector3d omega_B = v.head(3);  // First 3 elements
  
  // PERFECT gyro measurement (no noise, no bias)
  Eigen::Vector3d gyro_measured = omega_B;
  
  // 3. LINEAR ACCELERATION (Accelerometer)
  // Get rotation matrix from quaternion
  const math::RotationMatrixd R_WB(quat);
  
  // Gravity vector in world frame
  Eigen::Vector3d gravity_W(0, 0, gravity_);
  
  // Transform gravity to body frame (this is what accelerometer measures at rest)
  Eigen::Vector3d accel_B = R_WB.inverse() * gravity_W;
  
  // PERFECT accelerometer measurement (no noise, no bias)
  Eigen::Vector3d accel_measured = accel_B;
  
  // 4. POSITION (from quaternion state)
  // For floating body: q = [quat(4), position(3)]
  Eigen::Vector3d position_W = q.tail(3);  // Last 3 elements of q
  
  // PERFECT barometer measurement (no noise)
  double altitude_measured = position_W(2);
  
  // ========================================================================
  // PACK INTO 7-ELEMENT OUTPUT VECTOR (PERFECT MEASUREMENTS!)
  // ========================================================================
  Eigen::VectorXd imu_data(7);
  imu_data << accel_measured,        // [0-2] perfect accelerometer
              gyro_measured,         // [3-5] perfect gyroscope
              altitude_measured;     // [6] perfect barometer
  
  discrete_state->get_mutable_vector(0).SetFromVector(imu_data);
}

void ImuSensor::CalcImuOutput(const Context<double>& context,
                               BasicVector<double>* output) const {
  // Output the stored discrete state (last IMU reading)
  output->SetFromVector(context.get_discrete_state(0).CopyToVector());
}

}  // namespace systems
}  // namespace drake