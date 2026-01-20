#include "quadcopter_controller.h"

#include "drake/math/rigid_transform.h"

namespace drake {
namespace systems {

QuadcopterController::QuadcopterController(
    const multibody::MultibodyPlant<double>* plant,
    const multibody::RigidBody<double>* drone_body)
    : plant_(plant), drone_body_(drone_body) {
  this->DeclareVectorInputPort("thrust_commands", 4);
  this->DeclareAbstractOutputPort(
      "spatial_forces",
      &QuadcopterController::CalcSpatialForces);
}

void QuadcopterController::CalcSpatialForces(
    const Context<double>& context,
    std::vector<multibody::ExternallyAppliedSpatialForce<double>>* output) const {
  const auto& thrust_input = this->get_input_port(0).Eval(context);

  // Rotor positions (cross configuration)
  const double arm_length = 0.15;
  std::vector<Eigen::Vector3d> rotor_positions = {
      {arm_length, 0, 0},   // Front (along +X)
      {-arm_length, 0, 0},  // Back (along -X)
      {0, arm_length, 0},   // Left (along +Y)
      {0, -arm_length, 0}   // Right (along -Y)
  };

  output->clear();
  output->reserve(4);

  for (int i = 0; i < 4; ++i) {
    multibody::ExternallyAppliedSpatialForce<double> force;
    force.body_index = drone_body_->index();
    force.p_BoBq_B = rotor_positions[i];
    force.F_Bq_W = multibody::SpatialForce<double>(
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d(0, 0, thrust_input[i])
    );
    output->push_back(force);
  }
}

}  // namespace systems
}  // namespace drake
