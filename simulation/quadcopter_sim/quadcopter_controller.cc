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
  // Input port 0: 4-element control vector (thrust + angle setpoints)
  this->DeclareVectorInputPort("control_input", 4);
  
  // Input port 1: plant state (for reading drone orientation and rates)
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
  
  // ========================================================================
  // READ INPUTS
  // ========================================================================
  
  // Control inputs from user (port 0)
  const auto& u = this->get_input_port(0).Eval(context);
  const double total_thrust = u(0);         // Total thrust (N)
  const double target_roll = u(1);          // Target roll angle (rad)
  const double target_pitch = u(2);         // Target pitch angle (rad)
  const double target_yaw = u(3);           // Target yaw angle (rad)

  // Plant state (port 1)
  const auto& x = this->get_input_port(1).Eval(context);
  const int nq = plant_->num_positions();
  const int nv = plant_->num_velocities();
  
  Eigen::VectorXd q = x.head(nq);
  Eigen::VectorXd v = x.tail(nv);
  
  // Extract quaternion and create rotation matrix
  Eigen::Quaterniond quat(q(0), q(1), q(2), q(3));
  
  // Safety check: ensure quaternion is valid
  const double quat_norm = quat.norm();
  if (quat_norm < 0.1) {
    // Invalid quaternion - drone has flipped catastrophically
    // Set all forces to zero to prevent further damage
    auto& output = 
        output_abstract->get_mutable_value<std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>>();
    output.clear();
    return;
  }
  
  quat.normalize();
  const math::RotationMatrix<double> R_WB(quat);
  
  // Extract current orientation as Euler angles
  const math::RollPitchYaw<double> rpy(R_WB);
  const double current_roll = rpy.roll_angle();
  const double current_pitch = rpy.pitch_angle();
  const double current_yaw = rpy.yaw_angle();
  
  // Extract current angular rates (body frame)
  const Eigen::Vector3d omega = v.head(3);
  const double current_roll_rate = omega(0);
  const double current_pitch_rate = omega(1);
  const double current_yaw_rate = omega(2);
  
  // ========================================================================
  // CASCADED CONTROL: OUTER LOOP (Angle → Desired Rate)
  // ========================================================================
  // This is the stabilization magic!
  // When target_angle = 0 and current_angle ≠ 0:
  //   → angle_error is negative
  //   → desired_rate is negative
  //   → drone rotates back toward level
  
  // Compute angle errors
  double roll_error = target_roll - current_roll;
  double pitch_error = target_pitch - current_pitch;
  double yaw_error = target_yaw - current_yaw;
  
  // Normalize yaw error to [-π, π]
  while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
  while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
  
  // Outer loop: Convert angle errors to desired rates (P control)
  // These are the rates we WANT in order to correct the angle error
  const double desired_roll_rate = kp_angle_roll_ * roll_error;
  const double desired_pitch_rate = kp_angle_pitch_ * pitch_error;
  const double desired_yaw_rate = kp_angle_yaw_ * yaw_error;
  
  // ========================================================================
  // CASCADED CONTROL: INNER LOOP (Rate → Torque)
  // ========================================================================
  // Now we use PD control to achieve the desired rates computed above
  
  // Compute rate errors
  const double roll_rate_error = desired_roll_rate - current_roll_rate;
  const double pitch_rate_error = desired_pitch_rate - current_pitch_rate;
  const double yaw_rate_error = desired_yaw_rate - current_yaw_rate;
  
  // Inner loop: PD control on rates to generate torques
  // P term: proportional to rate error (main corrective action)
  // D term: damping based on current rate (prevents overshoot/oscillation)
  // 
  // Note: Negative D term because we want to oppose current motion
  // This creates critical damping for fastest stop without overshoot
  const double roll_torque = kp_rate_roll_ * roll_rate_error 
                           - kd_rate_roll_ * current_roll_rate;
  const double pitch_torque = kp_rate_pitch_ * pitch_rate_error 
                            - kd_rate_pitch_ * current_pitch_rate;
  const double yaw_torque = kp_rate_yaw_ * yaw_rate_error 
                          - kd_rate_yaw_ * current_yaw_rate;
  
  // ========================================================================
  // THRUST MIXING: Torques → Individual Rotor Thrusts
  // X-CONFIGURATION MIXING with YAW compensation
  // ========================================================================
  
  const double arm_length = 0.15;  // meters
  
  // Baseline: equal thrust on all rotors
  const double base_thrust_per_rotor = total_thrust / 4.0;
  
  // X-CONFIGURATION MIXING MATRIX
  // For X-config at 45°, effective arm length is arm_length/sqrt(2) per axis
  const double arm_eff = arm_length * 0.7071;  // 1/sqrt(2)
  
  // Convert torques to thrust differentials
  // In X-config, each motor contributes to BOTH pitch and roll
  const double pitch_contribution = pitch_torque / (2.0 * arm_eff);
  const double roll_contribution = roll_torque / (2.0 * arm_eff);
  
  // Yaw mixing: simulate counter-rotating propellers
  // In real quad: Front-Right & Back-Left spin CW, Front-Left & Back-Right spin CCW
  // Yaw torque distributed to maintain zero net angular momentum
  const double yaw_contribution = yaw_torque / 4.0;
  
  // Apply mixing matrix for X-configuration:
  // Motor layout after 45° rotation:
  //   front (+X) = Blue = Front-Right diagonal (CW prop)
  //   back (-X) = Green = Back-Left diagonal (CW prop)  
  //   left (+Y) = Red = Front-Left diagonal (CCW prop)
  //   right (-Y) = Yellow = Back-Right diagonal (CCW prop)
  
  // Blue (Front-Right): +pitch, +roll, -yaw (CW prop)
  double f_front = base_thrust_per_rotor + pitch_contribution + roll_contribution - yaw_contribution;
  
  // Green (Back-Left): -pitch, -roll, -yaw (CW prop)
  double f_back = base_thrust_per_rotor - pitch_contribution - roll_contribution - yaw_contribution;
  
  // Red (Front-Left): +pitch, -roll, +yaw (CCW prop)
  double f_left = base_thrust_per_rotor + pitch_contribution - roll_contribution + yaw_contribution;
  
  // Yellow (Back-Right): -pitch, +roll, +yaw (CCW prop)
  double f_right = base_thrust_per_rotor - pitch_contribution + roll_contribution + yaw_contribution;
  
  // Clamp to physical limits
  const double max_single_rotor = total_thrust * 0.9;
  f_front = std::clamp(f_front, 0.0, max_single_rotor);
  f_back  = std::clamp(f_back, 0.0, max_single_rotor);
  f_left  = std::clamp(f_left, 0.0, max_single_rotor);
  f_right = std::clamp(f_right, 0.0, max_single_rotor);
  
  // ========================================================================
  // APPLY FORCES TO MULTIBODY SYSTEM
  // ========================================================================
  
  const std::vector<Eigen::Vector3d> rotor_positions = {
      {arm_length, 0.0, 0.0},    // front
      {-arm_length, 0.0, 0.0},   // back
      {0.0, arm_length, 0.0},    // left
      {0.0, -arm_length, 0.0}    // right
  };
  
  auto& output = 
      output_abstract->get_mutable_value<std::vector<drake::multibody::ExternallyAppliedSpatialForce<double>>>();
  
  output.clear();
  output.reserve(5);
  
  // Apply rotor forces (transformed from body to world frame)
  auto push_rotor = [&](const Eigen::Vector3d& p_B, double fz) {
    drake::multibody::ExternallyAppliedSpatialForce<double> sf;
    sf.body_index = drone_body_->index();
    sf.p_BoBq_B = p_B;
    
    // Force in body +Z, rotated to world frame
    Eigen::Vector3d F_B(0.0, 0.0, fz);
    Eigen::Vector3d F_W = R_WB * F_B;
    
    sf.F_Bq_W = drake::multibody::SpatialForce<double>(
        Eigen::Vector3d::Zero(),
        F_W);
    output.push_back(sf);
  };
  
  push_rotor(rotor_positions[0], f_front);
  push_rotor(rotor_positions[1], f_back);
  push_rotor(rotor_positions[2], f_left);
  push_rotor(rotor_positions[3], f_right);
  
  // Apply yaw torque as pure moment
  if (std::abs(yaw_torque) > 1e-12) {
    drake::multibody::ExternallyAppliedSpatialForce<double> tau;
    tau.body_index = drone_body_->index();
    tau.p_BoBq_B = Eigen::Vector3d::Zero();
    
    Eigen::Vector3d M_B(0.0, 0.0, yaw_torque);
    Eigen::Vector3d M_W = R_WB * M_B;
    
    tau.F_Bq_W = drake::multibody::SpatialForce<double>(M_W, Eigen::Vector3d::Zero());
    output.push_back(tau);
  }
}

}  // namespace systems
}  // namespace drake