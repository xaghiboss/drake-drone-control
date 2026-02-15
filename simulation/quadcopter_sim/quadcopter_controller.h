#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/rigid_body.h"

namespace drake {
namespace systems {

// ============================================================================
// QuadcopterController - IMU-based Cascaded Attitude + Altitude Controller
// ============================================================================
// This controller implements cascaded PD control for quadcopters using
// realistic sensor data (IMU) instead of direct plant access.
//
// CONTROL LOOPS:
// 1. Altitude Loop: target_altitude → thrust (PD control)
// 2. Attitude Loop: target_angles → torques (cascaded P + PD)
// 3. Motor Mixing: thrust + torques → individual motor commands
//
class QuadcopterController : public LeafSystem<double> {
 public:
  QuadcopterController(const drake::multibody::MultibodyPlant<double>* plant,
                       const drake::multibody::RigidBody<double>* drone_body);

 private:
  std::unique_ptr<drake::AbstractValue> AllocateSpatialForces() const;

  // ========================================================================
  // INPUT PORTS
  // ========================================================================
  // 
  // Input port 0: "control_input" (5 elements) ← CHANGED FROM 4!
  //   [0]    target altitude (m) - for altitude hold PD controller
  //   [1]    desired roll angle (rad)
  //   [2]    desired pitch angle (rad)
  //   [3]    desired yaw angle (rad)
  //   [4]    altitude_mode (0.0 = manual thrust, 1.0 = auto-hover)
  //
  // Input port 1: "imu_state" (13 elements)
  //   [0-3]  Quaternion orientation (w, x, y, z)
  //   [4-6]  Angular velocity (body frame, rad/s)
  //   [7-9]  Linear acceleration (body frame, m/s²)
  //   [10-12] Position (world frame, m) - element [12] is altitude
  //
  // ========================================================================
  // OUTPUT PORT
  // ========================================================================
  // 
  // Output port 0: "spatial_forces" (AbstractValue)
  //   - Vector of ExternallyAppliedSpatialForce<double>
  //   - Contains 4 rotor thrust forces + 1 yaw moment
  //
  // ========================================================================
  
  void CalcSpatialForces(
      const Context<double>& context,
      drake::AbstractValue* output) const;

  const drake::multibody::MultibodyPlant<double>* plant_{nullptr};
  const drake::multibody::RigidBody<double>* drone_body_{nullptr};
  
  // ========================================================================
  // CONTROLLER GAINS (REALISTIC BETAFLIGHT VALUES)
  // ========================================================================
  
  // ALTITUDE LOOP: Altitude → Thrust
  const double kp_altitude_ = 3.0;       // Increased from 1.5
  const double kd_altitude_ = 2.0;       // Increased from 1.0
  
  // Filtering (keep these)
  const double alpha_altitude_ = 0.9;
  const double alpha_velocity_ = 0.9;
  const double altitude_deadband_ = 0.1;
  
  const double kp_angle_roll_ = 25.0;    // Was 3.0 → 5x higher
  const double kp_angle_pitch_ = 25.0;   // Was 3.0
  const double kp_angle_yaw_ = 8.0;      // Was 1.0 → 8x higher (yaw needs MORE!)
  
  // INNER LOOP: Rate → Torque (INCREASE THESE!)
  const double kp_rate_roll_ = 12.0;      // Was 1.2
  const double kd_rate_roll_ = 7.0;      // Was 0.6
  
  const double kp_rate_pitch_ = 12.0;     // Was 1.2
  const double kd_rate_pitch_ = 7.0;     // Was 0.6
  
  const double kp_rate_yaw_ = 10.0;      // Was 3.0 → Need even higher for yaw!
  const double kd_rate_yaw_ = 3.0;       // Was 1.0
  
  // Physical constants
  const double drone_mass_ = 0.5;
  const double gravity_ = 9.81;
  const double hover_thrust_ = drone_mass_ * gravity_;
  
  // Internal state for altitude control
  mutable double last_altitude_ = 0.0;
  mutable double last_time_ = -1.0;
  mutable double filtered_altitude_ = 0.0;   // ← ADD: filtered altitude measurement
  mutable double filtered_velocity_ = 0.0;
  mutable bool altitude_initialized_ = false;
};

}  // namespace systems
}  // namespace drake