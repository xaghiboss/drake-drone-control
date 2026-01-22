// quadcopter_model.cc
// Modified to create a rigid-body quadcopter with a realistic spatial inertia
// (central fuselage + four point-mass motors). Visual geometry is attached
// to that rigid body. This allows applied motor thrusts at rotor mount points
// to produce torques (roll/pitch/yaw) on the body.

#include "quadcopter_model.h"

#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/rotational_inertia.h"  // <- corrected include
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/spatial_inertia.h"    // <- correct include path for SpatialInertia

using drake::math::RigidTransformd;
using drake::math::RotationMatrix;
using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialInertia;
using drake::multibody::RotationalInertia;         // RotationalInertia lives in drake::multibody
using drake::geometry::SceneGraph;
using drake::AutoDiffXd;

namespace drake {

const multibody::RigidBody<double>& AddQuadcopterModel(
    multibody::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {

  // Physical parameters (tweak as desired)
  const double m_center = 0.30;   // kg, central fuselage
  const double m_motor = 0.05;    // kg, each motor + prop + arm mass
  const double arm_length = 0.15; // m, distance from center to rotor
  const double body_x = 0.18;     // m, fuselage X size (approx)
  const double body_y = 0.18;     // m, fuselage Y size (approx)
  const double body_z = 0.06;     // m, fuselage Z thickness (approx)

  const double total_mass = m_center + 4.0 * m_motor;

  // Central fuselage approximate inertia (box about its centroid):
  // Ixx = 1/12 * m * (y^2 + z^2), etc.
  const double Ixx_center = (1.0 / 12.0) * m_center * (body_y*body_y + body_z*body_z);
  const double Iyy_center = (1.0 / 12.0) * m_center * (body_x*body_x + body_z*body_z);
  const double Izz_center = (1.0 / 12.0) * m_center * (body_x*body_x + body_y*body_y);

  // Motor positions (body frame)
  const Eigen::Vector3d p_front( arm_length, 0.0, 0.0);
  const Eigen::Vector3d p_back( -arm_length, 0.0, 0.0);
  const Eigen::Vector3d p_left( 0.0,  arm_length, 0.0);
  const Eigen::Vector3d p_right(0.0, -arm_length, 0.0);

  // Start with central inertia
  double Ixx = Ixx_center;
  double Iyy = Iyy_center;
  double Izz = Izz_center;

  // Add point-mass contributions from motors (parallel-axis theorem)
  auto add_point_mass_inertia = [&](const Eigen::Vector3d& p, double m) {
    Ixx += m * (p.y()*p.y() + p.z()*p.z());
    Iyy += m * (p.x()*p.x() + p.z()*p.z());
    Izz += m * (p.x()*p.x() + p.y()*p.y());
  };

  add_point_mass_inertia(p_front, m_motor);
  add_point_mass_inertia(p_back,  m_motor);
  add_point_mass_inertia(p_left,  m_motor);
  add_point_mass_inertia(p_right, m_motor);

  // Build a RotationalInertia and SpatialInertia about the body origin (Bcm = body COM)
  // Use the constructor that accepts diagonal moments + products (use zeros for products).
  RotationalInertia<double> J(Ixx, Iyy, Izz, 0.0, 0.0, 0.0);
  SpatialInertia<double> M_Bcm = SpatialInertia<double>::MakeFromCentralInertia(
      total_mass, Eigen::Vector3d::Zero(), J);

  // Add the rigid body with computed spatial inertia
  const auto& body = plant->AddRigidBody("quadcopter_body", M_Bcm);

  // ===== Register collision geometry =====
  // Add a simplified collision box covering the arms/central body so the vehicle collides
  // with the ground and other collision geometry. Use Coulomb friction.
  const double collision_box_x = arm_length * 2.0;
  const double collision_box_y = arm_length * 2.0;
  const double collision_box_z = body_z;
  plant->RegisterCollisionGeometry(
      body,
      RigidTransformd::Identity(),
      geometry::Box(collision_box_x, collision_box_y, collision_box_z),
      "body_collision",
      multibody::CoulombFriction<double>(0.5, 0.5));

  // Small collision spheres at rotor mounts to better capture contact around arms.
  const double rotor_collision_r = 0.03;
  plant->RegisterCollisionGeometry(body, RigidTransformd(p_front),
      geometry::Sphere(rotor_collision_r), "rotor_collision_front",
      multibody::CoulombFriction<double>(0.5, 0.5));
  plant->RegisterCollisionGeometry(body, RigidTransformd(p_back),
      geometry::Sphere(rotor_collision_r), "rotor_collision_back",
      multibody::CoulombFriction<double>(0.5, 0.5));
  plant->RegisterCollisionGeometry(body, RigidTransformd(p_left),
      geometry::Sphere(rotor_collision_r), "rotor_collision_left",
      multibody::CoulombFriction<double>(0.5, 0.5));
  plant->RegisterCollisionGeometry(body, RigidTransformd(p_right),
      geometry::Sphere(rotor_collision_r), "rotor_collision_right",
      multibody::CoulombFriction<double>(0.5, 0.5));

  // ===== Visual geometry attached to the body (top plate) =====
  const double body_size = 0.09;  // for visual proportions
  const double top_plate_z = body_z / 2.0;  // small offset for top plate

  // Add simple visual geometry (top plate & 4 arms)
  // Colors are RGBA vectors
  using drake::Vector4;
  plant->RegisterVisualGeometry(
      body, RigidTransformd(Eigen::Vector3d(0, 0, top_plate_z)),
      geometry::Box(body_size, body_size, 0.006),
      "top_plate", Vector4<double>(0.1, 0.1, 0.1, 1.0));

  // Cross arms (visual only)
  const double arm_width = 0.02;
  const double arm_length_visual = arm_length;
  // Front (+X)
  plant->RegisterVisualGeometry(
      body, RigidTransformd(Eigen::Vector3d(arm_length_visual/2.0, 0, 0)),
      geometry::Box(arm_length_visual, arm_width, 0.004),
      "arm_x_pos", Vector4<double>(0.3, 0.3, 0.3, 1.0));
  // Back (-X)
  plant->RegisterVisualGeometry(
      body, RigidTransformd(Eigen::Vector3d(-arm_length_visual/2.0, 0, 0)),
      geometry::Box(arm_length_visual, arm_width, 0.004),
      "arm_x_neg", Vector4<double>(0.4, 0.4, 0.4, 1.0));
  // Left (+Y)
  plant->RegisterVisualGeometry(
      body, RigidTransformd(Eigen::Vector3d(0, arm_length_visual/2.0, 0)),
      geometry::Box(arm_width, arm_length_visual, 0.004),
      "arm_y_pos", Vector4<double>(0.3, 0.3, 0.3, 1.0));
  // Right (-Y)
  plant->RegisterVisualGeometry(
      body, RigidTransformd(Eigen::Vector3d(0, -arm_length_visual/2.0, 0)),
      geometry::Box(arm_width, arm_length_visual, 0.004),
      "arm_y_neg", Vector4<double>(0.4, 0.4, 0.4, 1.0));

  // Optional: add small spheres at rotor mounts to visualize rotor positions
  const double rotor_vis_r = 0.02;
  plant->RegisterVisualGeometry(body, RigidTransformd(p_front),
      geometry::Sphere(rotor_vis_r), "rotor_front", Vector4<double>(0.8, 0.1, 0.1, 1.0));
  plant->RegisterVisualGeometry(body, RigidTransformd(p_back),
      geometry::Sphere(rotor_vis_r), "rotor_back", Vector4<double>(0.8, 0.1, 0.1, 1.0));
  plant->RegisterVisualGeometry(body, RigidTransformd(p_left),
      geometry::Sphere(rotor_vis_r), "rotor_left", Vector4<double>(0.1, 0.1, 0.8, 1.0));
  plant->RegisterVisualGeometry(body, RigidTransformd(p_right),
      geometry::Sphere(rotor_vis_r), "rotor_right", Vector4<double>(0.1, 0.1, 0.8, 1.0));

  // Return reference to the created rigid body (works because plant owns it)
  return body;
}

void AddGround(multibody::MultibodyPlant<double>* plant,
               geometry::SceneGraph<double>* scene_graph) {
  // Simple ground plane visual and collision
  using drake::math::RigidTransformd;
  using drake::Vector4;

  const double grid_spacing = 0.25;
  const int grid_count = 10;
  const double line_width = 0.01;

  for (int i = -grid_count; i <= grid_count; ++i) {
    // Lines along X axis
    plant->RegisterVisualGeometry(
        plant->world_body(),
        RigidTransformd(Eigen::Vector3d(0, i * grid_spacing, -0.004)),
        geometry::Box(grid_spacing * grid_count * 2, line_width, 0.001),
        "grid_x_" + std::to_string(i),
        Vector4<double>(0.15, 0.3, 0.15, 1.0));

    // Lines along Y axis
    plant->RegisterVisualGeometry(
        plant->world_body(),
        RigidTransformd(Eigen::Vector3d(i * grid_spacing, 0, -0.004)),
        geometry::Box(grid_spacing * grid_count * 2, line_width, 0.001),
        "grid_y_" + std::to_string(i),
        Vector4<double>(0.15, 0.3, 0.15, 1.0));
  }
}

}  // namespace drake