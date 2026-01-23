#pragma once

#include <vector>
#include <memory>

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/common/value.h"

namespace drake {
namespace systems {

class QuadcopterController : public LeafSystem<double> {
 public:
  // Constructs controller given pointer to plant and the drone rigid body.
  QuadcopterController(const drake::multibody::MultibodyPlant<double>* plant,
                       const drake::multibody::RigidBody<double>* drone_body);

 private:
  // Allocator for the output port
  std::unique_ptr<drake::AbstractValue> AllocateSpatialForces() const;

  // Create the output spatial forces from the input control vector.
  // Input port 0 is a vector of length 4:
  //   [0]    total thrust (N) - distributed equally to all 4 rotors
  //   [1]    desired roll angle (rad) - AUTO-LEVELS to 0 when input = 0
  //   [2]    desired pitch angle (rad) - AUTO-LEVELS to 0 when input = 0
  //   [3]    desired yaw angle (rad)
  //
  // Input port 1 is the plant state (auto-connected)
  //
  // This implements CASCADED ANGLE + RATE CONTROL (like Stabilize/Angle mode):
  // - Outer loop: P controller on angle error → desired rate
  // - Inner loop: PD controller on rate error → torque
  // - When input = 0: target angle = 0, drone auto-levels and stops
  void CalcSpatialForces(
      const Context<double>& context,
      drake::AbstractValue* output) const;

  const drake::multibody::MultibodyPlant<double>* plant_{nullptr};
  const drake::multibody::RigidBody<double>* drone_body_{nullptr};
  
  // ========================================================================
  // CASCADED CONTROLLER GAINS (tuned for MAXIMUM RESPONSIVENESS)
  // ========================================================================
  
  // OUTER LOOP: Angle → Rate (Proportional only)
  // Higher gain = faster correction when tilted
  // Tuned for aggressive response without overshoot
  const double kp_angle_roll_ = 8.0;     // Roll angle to rate (was 4.0)
  const double kp_angle_pitch_ = 8.0;    // Pitch angle to rate (was 4.0)
  const double kp_angle_yaw_ = 3.0;      // Yaw angle to rate (was 2.0)
  
  // INNER LOOP: Rate → Torque (PD control)
  // Higher kp = more aggressive torque response
  // Higher kd = more damping (prevents oscillation)
  // Tuned for critical damping (fastest stop without overshoot)
  const double kp_rate_roll_ = 0.15;     // Roll rate P gain (was 0.08)
  const double kd_rate_roll_ = 0.008;    // Roll rate D gain (was 0.002)
  
  const double kp_rate_pitch_ = 0.15;    // Pitch rate P gain (was 0.08)
  const double kd_rate_pitch_ = 0.008;   // Pitch rate D gain (was 0.002)
  
  const double kp_rate_yaw_ = 0.08;      // Yaw rate P gain (was 0.05)
  const double kd_rate_yaw_ = 0.004;     // Yaw rate D gain (was 0.002)
};

}  // namespace systems
}  // namespace drake