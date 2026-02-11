#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"
#include <Eigen/Dense>

namespace drake {
namespace systems {

class ObstacleDetector : public LeafSystem<double> {
 public:
  ObstacleDetector();
  ~ObstacleDetector() override = default;

  // Manual function to process control input with depth image
  Eigen::Vector4d ProcessControl(
      const sensors::ImageDepth32F& depth_image,
      const Eigen::Vector4d& desired_control) const;

 private:
  void ComputeSafeVelocity(const Context<double>& context,
                           BasicVector<double>* output) const;
  
  void CheckObstacles(const Context<double>& context) const;
  
  const double safety_distance_ = 1.5;
};

}  // namespace systems
}  // namespace drake