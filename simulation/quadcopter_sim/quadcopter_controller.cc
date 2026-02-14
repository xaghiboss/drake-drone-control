#include "quadcopter_controller.h"

#include <algorithm>
#include <functional>

#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"

namespace drake {
namespace systems {

QuadcopterController::QuadcopterController(
    const drake::multibody::MultibodyPlant<double>* plant,
    const drake::multibody::RigidBody<double>* drone_body)
    : plant_(plant), drone_body_(drone_body) {
  
  // Input port 0: 5-element control vector ← CHANGED FROM 4!
  // [target_altitude, roll, pitch, yaw, altitude_mode]
  this->DeclareVectorInputPort("control_input", 5);
  
  // Input port 1: 13-element IMU state vector
  this->DeclareVectorInputPort("imu_state", 13);
  
  // Output: spatial forces
  this->DeclareAbstractOutputPort(
      "spatial_forces",
      [this]() { return this->AllocateSpatialForces(); },
      [this](const Context<double>& context, drake::AbstractValue* output) {
        this->CalcSpatialForces(context, output);
      });
}

std::unique_ptr<drake::AbstractValue>
QuadcopterController::AllocateSpatialForces() const {
  return drake::AbstractValue::Make(
      std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>());
}

void QuadcopterController::CalcSpatialForces(
    const Context<double>& context,
    drake::AbstractValue* output_abstract) const {
  
  // ========================================================================
  // READ INPUTS
  // ========================================================================
  
  // Control inputs from user (port 0) - NOW 5 ELEMENTS!
  const auto& u = this->get_input_port(0).Eval(context);
  const double target_altitude = u(0);      // Target altitude (m)
  const double target_roll = u(1);          // Target roll angle (rad)
  const double target_pitch = u(2);         // Target pitch angle (rad)
  const double target_yaw = u(3);           // Target yaw angle (rad)
  const double altitude_mode = u(4);        // 0.0 = manual, 1.0 = auto-hover

  // ========================================================================
  // IMU SENSOR DATA (port 1) - 13 ELEMENTS
  // ========================================================================
  const auto& imu = this->get_input_port(1).Eval(context);
  
  // [0-3] Quaternion orientation
  Eigen::Quaterniond quat(imu(0), imu(1), imu(2), imu(3));
  quat.normalize();
  
  // Safety check
  const double quat_norm = quat.norm();
  if (quat_norm < 0.1) {
    auto& output = 
        output_abstract->get_mutable_value<std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>>();
    output.clear();
    return;
  }
  
  // [4-6] Angular velocity (gyroscope, body frame, rad/s)
  const double current_roll_rate = imu(4);
  const double current_pitch_rate = imu(5);
  const double current_yaw_rate = imu(6);
  
  // [7-9] Linear acceleration (accelerometer, body frame, m/s²)
  const Eigen::Vector3d accel_body = imu.segment(7, 3);
  
  // [10-12] Position (world frame, m)
  const double current_altitude = imu(12);  // Z coordinate = altitude
  
  // ========================================================================
  // EXTRACT ORIENTATION FROM QUATERNION
  // ========================================================================
  const math::RotationMatrix<double> R_WB(quat);
  const math::RollPitchYaw<double> rpy(R_WB);
  const double current_roll = rpy.roll_angle();
  const double current_pitch = rpy.pitch_angle();
  const double current_yaw = rpy.yaw_angle();
  
  // Safety check: verify orientation is reasonable
  if (std::abs(current_roll) > M_PI / 2.5 || std::abs(current_pitch) > M_PI / 2.5) {
    auto& output = 
        output_abstract->get_mutable_value<std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>>();
    output.clear();
    return;
  }
  
   // ========================================================================
  // ALTITUDE CONTROL LOOP (SIMPLE FIX)
  // ========================================================================
  
  double total_thrust = 0.0;
  
  if (altitude_mode > 0.5) {  // Auto-hover enabled
    const double current_time = context.get_time();
    
    if (!altitude_initialized_) {
      filtered_altitude_ = current_altitude;
      last_altitude_ = current_altitude;
      last_time_ = current_time;
      filtered_velocity_ = 0.0;
      altitude_initialized_ = true;
      
      // SIMPLE FIX: Set initial thrust based on target altitude
      if (target_altitude < 0.15) {
        total_thrust = 0.0;  // Target is ground → zero thrust
      } else {
        total_thrust = hover_thrust_;  // Target is in air → hover thrust
      }
      
    } else {
      // Low-pass filter altitude
      filtered_altitude_ = alpha_altitude_ * filtered_altitude_ 
                         + (1.0 - alpha_altitude_) * current_altitude;
      
      const double dt = current_time - last_time_;
      
      // Compute error with deadband
      double altitude_error = target_altitude - filtered_altitude_;
      if (std::abs(altitude_error) < altitude_deadband_) {
        altitude_error = 0.0;
      }
      
      // Estimate velocity
      double vertical_velocity = 0.0;
      if (dt > 1e-6) {
        const double raw_velocity = (filtered_altitude_ - last_altitude_) / dt;
        filtered_velocity_ = alpha_velocity_ * filtered_velocity_ 
                           + (1.0 - alpha_velocity_) * raw_velocity;
        vertical_velocity = filtered_velocity_;
      }
      
      last_altitude_ = filtered_altitude_;
      last_time_ = current_time;
      
      // CRITICAL: Choose base thrust based on WHERE we're trying to be
      double base_thrust;
      if (target_altitude < 0.15) {
        // Trying to land/stay on ground
        base_thrust = 0.0;
      } else {
        // Flying in air
        base_thrust = hover_thrust_;
      }
      
      // PD correction
      const double altitude_correction = kp_altitude_ * altitude_error 
                                       - kd_altitude_ * vertical_velocity;
      
      total_thrust = base_thrust + altitude_correction;
      
      // Clamp (allow zero for landing!)
      total_thrust = std::clamp(total_thrust, 0.0, hover_thrust_ * 1.5);
    }
  } else {
    altitude_initialized_ = false;
    total_thrust = 0.0;
  }
  
  // ========================================================================
  // CASCADED ATTITUDE CONTROL: OUTER LOOP (Angle → Desired Rate)
  // ========================================================================
  
  // Compute angle errors
  double roll_error = target_roll - current_roll;
  double pitch_error = target_pitch - current_pitch;
  double yaw_error = target_yaw - current_yaw;
  
  // Normalize yaw error to [-π, π]
  while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
  while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
  
  // Outer loop: Convert angle errors to desired rates (P control)
  const double desired_roll_rate = kp_angle_roll_ * roll_error;
  const double desired_pitch_rate = kp_angle_pitch_ * pitch_error;
  const double desired_yaw_rate = kp_angle_yaw_ * yaw_error;
  
  // ========================================================================
  // CASCADED ATTITUDE CONTROL: INNER LOOP (Rate → Torque)
  // ========================================================================
  
  // Compute rate errors
  const double roll_rate_error = desired_roll_rate - current_roll_rate;
  const double pitch_rate_error = desired_pitch_rate - current_pitch_rate;
  const double yaw_rate_error = desired_yaw_rate - current_yaw_rate;
  
  // Inner loop: PD control on rates to generate torque commands
  const double roll_torque = kp_rate_roll_ * roll_rate_error 
                           - kd_rate_roll_ * current_roll_rate;
  const double pitch_torque = kp_rate_pitch_ * pitch_rate_error 
                            - kd_rate_pitch_ * current_pitch_rate;
  const double yaw_torque = kp_rate_yaw_ * yaw_rate_error 
                          - kd_rate_yaw_ * current_yaw_rate;
  
  // ========================================================================
  // DIFFERENTIAL THRUST MIXING - REAL DRONE STYLE
  // ========================================================================
  
  const double arm_length = 0.15;  // meters
  
  // Baseline: equal thrust on all rotors
  const double base_thrust_per_rotor = total_thrust / 4.0;
  
  // Convert torques to differential thrust amounts
  const double pitch_diff = pitch_torque / (2.0 * arm_length);
  const double roll_diff = roll_torque / (2.0 * arm_length);
  
  // REAL DRONE DIFFERENTIAL THRUST MIXING (X-configuration):
  double f_blue = base_thrust_per_rotor - pitch_diff - roll_diff;
  double f_red = base_thrust_per_rotor - pitch_diff + roll_diff;
  double f_yellow = base_thrust_per_rotor + pitch_diff - roll_diff;
  double f_green = base_thrust_per_rotor + pitch_diff + roll_diff;
  
  // Clamp to physical limits
  const double max_single_rotor = total_thrust * 0.9;
  f_blue = std::clamp(f_blue, 0.0, max_single_rotor);
  f_red = std::clamp(f_red, 0.0, max_single_rotor);
  f_yellow = std::clamp(f_yellow, 0.0, max_single_rotor);
  f_green = std::clamp(f_green, 0.0, max_single_rotor);
  
  // ========================================================================
  // APPLY FORCES TO MULTIBODY SYSTEM
  // ========================================================================
  
  const std::vector<Eigen::Vector3d> rotor_positions = {
      {arm_length, -arm_length, 0.0},    // Blue - Front-Right diagonal
      {arm_length, arm_length, 0.0},     // Red - Front-Left diagonal
      {-arm_length, -arm_length, 0.0},   // Yellow - Back-Right diagonal
      {-arm_length, arm_length, 0.0}     // Green - Back-Left diagonal
  };
  
  auto& output = 
      output_abstract->get_mutable_value<std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>>();
  
  output.clear();
  output.reserve(4);
  
  // Apply rotor forces (transformed from body to world frame)
  auto push_rotor = [&](const Eigen::Vector3d& p_B, double fz) {
    drake::multibody::ExternallyAppliedSpatialForce<double> sf;
    sf.body_index = drone_body_->index();
    sf.p_BoBq_B = p_B;
    
    Eigen::Vector3d F_B(0.0, 0.0, fz);
    Eigen::Vector3d F_W = R_WB * F_B;
    
    sf.F_Bq_W = drake::multibody::SpatialForce<double>(
        Eigen::Vector3d::Zero(),
        F_W);
    output.push_back(sf);
  };
  
  push_rotor(rotor_positions[0], f_blue);
  push_rotor(rotor_positions[1], f_red);
  push_rotor(rotor_positions[2], f_yellow);
  push_rotor(rotor_positions[3], f_green);
  
  // Apply yaw torque as pure moment
  const double max_yaw_torque = 0.5;
  const double yaw_torque_clamped = std::clamp(yaw_torque, -max_yaw_torque, max_yaw_torque);

  if (std::abs(yaw_torque_clamped) > 1e-12) {
    drake::multibody::ExternallyAppliedSpatialForce<double> yaw_moment;
    yaw_moment.body_index = drone_body_->index();
    yaw_moment.p_BoBq_B = Eigen::Vector3d::Zero();
    
    Eigen::Vector3d M_B(0.0, 0.0, yaw_torque_clamped);
    Eigen::Vector3d M_W = R_WB * M_B;

    yaw_moment.F_Bq_W = drake::multibody::SpatialForce<double>(
        M_W,                      
        Eigen::Vector3d::Zero()
    );
    
    output.push_back(yaw_moment);
  }
}

}  // namespace systems
}  // namespace drake