#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

// Converts keyboard commands to desired body-frame velocities
class VelocityCommand : public LeafSystem<double> {
 public:
  VelocityCommand();
  ~VelocityCommand() override = default;

 private:
  void ConvertToVelocity(const Context<double>& context,
                         BasicVector<double>* output) const;
  
  const double max_velocity_ = 1.0;  // m/s
};

}  // namespace systems
}  // namespace drake