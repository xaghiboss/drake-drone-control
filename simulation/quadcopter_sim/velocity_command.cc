#include "velocity_command.h"
#include <iostream>

namespace drake {
namespace systems {

VelocityCommand::VelocityCommand() {
  // Input: keyboard state (from your existing keyboard input system)
  // For now, we'll create a simple mapping
  
  // Output: desired velocity [vx, vy, vz] in body frame
  this->DeclareVectorOutputPort("desired_velocity", 3,
                                &VelocityCommand::ConvertToVelocity);
  
  std::cout << "VelocityCommand created (max velocity: " 
            << max_velocity_ << " m/s)" << std::endl;
}

void VelocityCommand::ConvertToVelocity(
    const Context<double>& context,
    BasicVector<double>* output) const {
  
  // For now, output a constant test velocity
  // We'll connect this to keyboard later
  Eigen::Vector3d velocity;
  velocity << 0.5, 0.0, 0.0;  // 0.5 m/s forward
  
  output->SetFromVector(velocity);
}

}  // namespace systems
}  // namespace drake