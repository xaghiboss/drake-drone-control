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
  // Input port 0: 10-element control vector (thrust + attitude setpoints + rates)
  this->DeclareVectorInputPort("control_input", 10);
  
  // Input port 1: plant state (for reading drone orientation and velocity)
  this->DeclareVectorInputPort("plant_state", 
                               plant->num_positions() + plant->num_velocities());
  
  // Output: abstract vector of spatial forces applied externally.
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
  
  // Read control input from port 0
  const auto& u = this->get_input_port(0).Eval(context);

  // Extract control inputs
  const double total_thrust = u(0);        // Total thrust to distribute
  const double desired_roll = u(1);        // Desired roll angle (rad)
  const double desired_pitch = u(2);       // Desired pitch angle (rad)
  const double desired_yaw = u(3);         // Desired yaw angle (rad)
  const double desired_roll_rate = u(4);   // Desired roll rate (rad/s)
  const double desired_pitch_rate = u(5);  // Desired pitch rate (rad/s)
  const double desired_yaw_rate = u(6);    // Desired yaw rate (rad/s)

  // Read plant state from port 1
  const auto& x = this->get_input_port(1).Eval(context);
  
  const int nq = plant_->num_positions();
  const int nv = plant_->num_velocities();
  
  // Extract position and velocity from state
  Eigen::VectorXd q = x.head(nq);
  Eigen::VectorXd v = x.tail(nv);
  
  // For a floating body, the quaternion is at indices [0-3] and position at [4-6]
  // Velocities are angular [0-2] and linear [3-5]
  
  // Extract quaternion (w, x, y, z format in Drake)
  Eigen::Quaterniond quat(q(0), q(1), q(2), q(3));
  quat.normalize();
  
  // Create rotation matrix from quaternion
  const math::RotationMatrix<double> R_WB(quat);
  
  // Extract roll-pitch-yaw from rotation matrix
  const math::RollPitchYaw<double> rpy(R_WB);
  const double current_roll = rpy.roll_angle();
  const double current_pitch = rpy.pitch_angle();
  const double current_yaw = rpy.yaw_angle();
  
  // Extract angular velocity (first 3 elements of velocity vector)
  const Eigen::Vector3d omega_WB = v.head(3);
  
  // Compute attitude errors
  double roll_error = desired_roll - current_roll;
  double pitch_error = desired_pitch - current_pitch;
  double yaw_error = desired_yaw - current_yaw;
  
  // Normalize yaw error to [-pi, pi]
  while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
  while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
  
  // Compute rate errors
  double roll_rate_error = desired_roll_rate - omega_WB(0);
  double pitch_rate_error = desired_pitch_rate - omega_WB(1);
  double yaw_rate_error = desired_yaw_rate - omega_WB(2);
  
  // PD control for attitude
  double roll_torque = kp_roll_ * roll_error + kd_roll_ * roll_rate_error;
  double pitch_torque = kp_pitch_ * pitch_error + kd_pitch_ * pitch_rate_error;
  double yaw_torque = kp_yaw_ * yaw_error + kd_yaw_ * yaw_rate_error;
  
  // Robot geometry
  const double arm_length = 0.15;
  
  // Distribute total thrust equally to all 4 rotors as baseline
  double base_thrust_per_rotor = total_thrust / 4.0;
  
  // Apply control torques through differential thrust
  // For cross configuration:
  // Roll (Mx): differential on left/right rotors
  // Pitch (My): differential on front/back rotors
  double delta_roll = roll_torque / arm_length;
  double delta_pitch = pitch_torque / arm_length;
  
  // Rotor thrust assignment for cross config
  double f_front = base_thrust_per_rotor - delta_pitch;
  double f_back  = base_thrust_per_rotor + delta_pitch;
  double f_left  = base_thrust_per_rotor + delta_roll;
  double f_right = base_thrust_per_rotor - delta_roll;
  
  // Clamp to non-negative and reasonable max
  const double max_single_rotor = total_thrust * 0.9;
  f_front = std::clamp(f_front, 0.0, max_single_rotor);
  f_back  = std::clamp(f_back, 0.0, max_single_rotor);
  f_left  = std::clamp(f_left, 0.0, max_single_rotor);
  f_right = std::clamp(f_right, 0.0, max_single_rotor);
  
  // Rotor positions in body frame: front, back, left, right
  const std::vector<Eigen::Vector3d> rotor_positions = {
      {arm_length, 0.0, 0.0},    // front (+X)
      {-arm_length, 0.0, 0.0},   // back (-X)
      {0.0, arm_length, 0.0},    // left (+Y)
      {0.0, -arm_length, 0.0}    // right (-Y)
  };
  
  // Get the concrete output vector
  auto& output = 
      output_abstract->get_mutable_value<std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>>();
  
  output.clear();
  output.reserve(5);
  
  // Helper to push rotor force - forces in BODY frame, rotated to world
  auto push_rotor = [&](const Eigen::Vector3d& p_B, double fz) {
    drake::multibody::ExternallyAppliedSpatialForce<double> sf;
    sf.body_index = drone_body_->index();
    sf.p_BoBq_B = p_B;  // Position in body frame
    
    // Force is in BODY +Z direction (rotor thrust up in body frame)
    Eigen::Vector3d F_B(0.0, 0.0, fz);  
    Eigen::Vector3d F_W = R_WB * F_B;    // Rotate to world frame
    
    sf.F_Bq_W = drake::multibody::SpatialForce<double>(
        Eigen::Vector3d::Zero(),  // No direct moment component
        F_W);                      // Force in world frame
    output.push_back(sf);
  };
  
  push_rotor(rotor_positions[0], f_front);
  push_rotor(rotor_positions[1], f_back);
  push_rotor(rotor_positions[2], f_left);
  push_rotor(rotor_positions[3], f_right);
  
  // Apply yaw torque as pure moment about body Z-axis
  if (std::abs(yaw_torque) > 1e-12) {
    drake::multibody::ExternallyAppliedSpatialForce<double> tau;
    tau.body_index = drone_body_->index();
    tau.p_BoBq_B = Eigen::Vector3d::Zero();
    
    // Moment in body frame, rotated to world frame
    Eigen::Vector3d M_B(0.0, 0.0, yaw_torque);
    Eigen::Vector3d M_W = R_WB * M_B;
    
    tau.F_Bq_W = drake::multibody::SpatialForce<double>(
        M_W,                       // Moment in world frame
        Eigen::Vector3d::Zero());  // No force component
    output.push_back(tau);
  }
}

}  // namespace systems
}  // namespace drake