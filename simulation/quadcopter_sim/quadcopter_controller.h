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
  // Input port 0 is a vector of length 10:
  //   [0]    total thrust (N) - distributed equally to all 4 rotors
  //   [1]    desired roll angle (rad)
  //   [2]    desired pitch angle (rad)
  //   [3]    desired yaw angle (rad)
  //   [4]    desired roll rate (rad/s) - optional, can be 0
  //   [5]    desired pitch rate (rad/s) - optional, can be 0
  //   [6]    desired yaw rate (rad/s) - optional, can be 0
  //   [7-9]  reserved for future use
  //
  // Input port 1 is the plant state (auto-connected)
  void CalcSpatialForces(
      const Context<double>& context,
      drake::AbstractValue* output) const;

  const drake::multibody::MultibodyPlant<double>* plant_{nullptr};
  const drake::multibody::RigidBody<double>* drone_body_{nullptr};
  
  // PID gains for attitude control
  const double kp_roll_ = 3.0;    // Proportional gain for roll
  const double kd_roll_ = 0.8;    // Derivative gain for roll
  const double kp_pitch_ = 3.0;   // Proportional gain for pitch
  const double kd_pitch_ = 0.8;   // Derivative gain for pitch
  const double kp_yaw_ = 2.0;     // Proportional gain for yaw
  const double kd_yaw_ = 0.5;     // Derivative gain for yaw
};

}  // namespace systems
}  // namespace drake