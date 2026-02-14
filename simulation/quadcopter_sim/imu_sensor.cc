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
  // EXTRACT STATE DIRECTLY FROM VECTORS (NO PLANT CONTEXT NEEDED!)
  // ========================================================================
  
  // 1. ORIENTATION (Quaternion) - directly from q
  Eigen::Quaterniond quat(q(0), q(1), q(2), q(3));
  quat.normalize();
  
  // Add small orientation noise (simulate gyro drift integration)
  static std::default_random_engine gen(std::random_device{}());
  std::normal_distribution<double> quat_noise(0.0, 0.001);
  
  Eigen::Quaterniond quat_noisy(
      quat.w() + quat_noise(gen),
      quat.x() + quat_noise(gen),
      quat.y() + quat_noise(gen),
      quat.z() + quat_noise(gen)
  );
  quat_noisy.normalize();
  
  // 2. ANGULAR VELOCITY (Gyroscope) - directly from v
  // For a floating body, v = [angular_velocity(3), linear_velocity(3)]
  const Eigen::Vector3d omega_B = v.head(3);  // First 3 elements
  
  // Add gyro noise + bias
  std::normal_distribution<double> gyro_noise(0.0, gyro_noise_stddev_);
  Eigen::Vector3d gyro_measured = omega_B +
      Eigen::Vector3d(gyro_noise(gen), gyro_noise(gen), gyro_noise(gen)) +
      Eigen::Vector3d(gyro_bias_, gyro_bias_, gyro_bias_);
  
  // 3. LINEAR ACCELERATION (Accelerometer)
  // We need to compute acceleration from velocity derivative
  // For simplicity in simulation, we'll approximate or use gravity + small noise
  
  // Get rotation matrix from quaternion
  const math::RotationMatrixd R_WB(quat);
  
  // Gravity vector in world frame
  Eigen::Vector3d gravity_W(0, 0, gravity_);
  
  // Transform gravity to body frame (this is what accelerometer measures at rest)
  Eigen::Vector3d accel_B = R_WB.inverse() * gravity_W;
  
  // Add noise + bias
  std::normal_distribution<double> accel_noise(0.0, accel_noise_stddev_);
  Eigen::Vector3d accel_measured = accel_B + 
      Eigen::Vector3d(accel_noise(gen), accel_noise(gen), accel_noise(gen)) +
      Eigen::Vector3d(accel_bias_, accel_bias_, accel_bias_);
  
  // 4. POSITION (from quaternion state)
  // For floating body: q = [quat(4), position(3)]
  Eigen::Vector3d position_W = q.tail(3);  // Last 3 elements of q
  
  // Add barometer noise to altitude (Z)
  std::normal_distribution<double> baro_noise(0.0, baro_noise_stddev_);
  Eigen::Vector3d position_measured = position_W;
  position_measured(2) += baro_noise(gen);  // Only Z has baro noise
  
  // ========================================================================
  // PACK INTO 13-ELEMENT OUTPUT VECTOR
  // ========================================================================
  Eigen::VectorXd imu_data(7);
  imu_data << accel_measured,
              gyro_measured,                                                    // [7-9]
              position_measured(2);                                                 // [10-12]
  
  discrete_state->get_mutable_vector(0).SetFromVector(imu_data);
}

void ImuSensor::CalcImuOutput(const Context<double>& context,
                               BasicVector<double>* output) const {
  // Output the stored discrete state (last IMU reading)
  output->SetFromVector(context.get_discrete_state(0).CopyToVector());
}

}  // namespace systems
}  // namespace drake