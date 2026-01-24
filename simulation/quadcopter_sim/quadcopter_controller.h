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
  // This implements CASCADED ANGLE + RATE CONTROL for X-CONFIGURATION:
  // - Outer loop: P controller on angle error → desired rate
  // - Inner loop: PD controller on rate error → torque
  // - Thrust mixing: ALL 4 MOTORS participate in pitch AND roll
  //
  // X-CONFIG MOTOR MIXING (quadcopter rotated 45°):
  //   Forward: Blue+Red increase, Yellow+Green decrease
  //   Backward: Blue+Red decrease, Yellow+Green increase  
  //   Left: Blue+Yellow increase, Red+Green decrease
  //   Right: Blue+Yellow decrease, Red+Green increase
  //
  // When input = 0: target angle = 0, drone auto-levels and stops
  void CalcSpatialForces(
      const Context<double>& context,
      drake::AbstractValue* output) const;

  const drake::multibody::MultibodyPlant<double>* plant_{nullptr};
  const drake::multibody::RigidBody<double>* drone_body_{nullptr};
  
  // ========================================================================
  // CASCADED CONTROLLER GAINS (tuned for STABLE yet RESPONSIVE flight)
  // ========================================================================
  
  // OUTER LOOP: Angle → Rate (Proportional only)
  // Balanced for quick response without instability at takeoff
  const double kp_angle_roll_ = 50.0;     // Reduced from 15.0 for stability
  const double kp_angle_pitch_ = 50.0;    // Reduced from 15.0 for stability
  const double kp_angle_yaw_ = 50.0;      // Keep moderate for yaw
  
  // INNER LOOP: Rate → Torque (PD control)
  // Balanced P and D for responsive yet stable control
  const double kp_rate_roll_ = 15.0;     // Reduced from 0.25
  const double kd_rate_roll_ = 5;     // Reduced from 0.015
  
  const double kp_rate_pitch_ = 15.0;    // Reduced from 0.25
  const double kd_rate_pitch_ = 5;    // Reduced from 0.015
  
  const double kp_rate_yaw_ = 5;      // Keep moderate
  const double kd_rate_yaw_ = 2.0;     // Keep moderate
};

}  // namespace systems
}  // namespace drake