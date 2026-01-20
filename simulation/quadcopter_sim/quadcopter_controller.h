#ifndef QUADCOPTER_CONTROLLER_H_
#define QUADCOPTER_CONTROLLER_H_

#include <vector>

#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

class QuadcopterController : public LeafSystem<double> {
 public:
  QuadcopterController(
      const multibody::MultibodyPlant<double>* plant,
      const multibody::RigidBody<double>* drone_body);

 private:
  void CalcSpatialForces(
      const Context<double>& context,
      std::vector<multibody::ExternallyAppliedSpatialForce<double>>* output) const;

  const multibody::MultibodyPlant<double>* plant_;
  const multibody::RigidBody<double>* drone_body_;
};

}  // namespace systems
}  // namespace drake

#endif  // QUADCOPTER_CONTROLLER_H_
