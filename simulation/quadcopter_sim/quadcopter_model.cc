// quadcopter_model.cc - FIXED with clear directional color coding
// Blue = Front (+X), Red = Left (+Y), Green = Back (-X), Yellow = Right (-Y)

#include "quadcopter_model.h"

#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/rotational_inertia.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/spatial_inertia.h"

using drake::math::RigidTransformd;
using drake::math::RotationMatrix;
using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialInertia;
using drake::multibody::RotationalInertia;
using drake::geometry::SceneGraph;
using drake::AutoDiffXd;

namespace drake {

const multibody::RigidBody<double>& AddQuadcopterModel(
    multibody::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {

  // Physical parameters
  const double m_center = 0.30;   // kg, central fuselage
  const double m_motor = 0.05;    // kg, each motor + prop + arm mass
  const double arm_length = 0.15; // m, distance from center to rotor
  const double body_x = 0.18;     // m, fuselage X size
  const double body_y = 0.18;     // m, fuselage Y size
  const double body_z = 0.06;     // m, fuselage Z thickness

  const double total_mass = m_center + 4.0 * m_motor;

  // Central fuselage inertia
  const double Ixx_center = (1.0 / 12.0) * m_center * (body_y*body_y + body_z*body_z);
  const double Iyy_center = (1.0 / 12.0) * m_center * (body_x*body_x + body_z*body_z);
  const double Izz_center = (1.0 / 12.0) * m_center * (body_x*body_x + body_y*body_y);

  // Motor positions (CROSS configuration)
  const Eigen::Vector3d p_front( arm_length, 0.0, 0.0);   // +X (FORWARD)
  const Eigen::Vector3d p_back( -arm_length, 0.0, 0.0);   // -X (BACKWARD)
  const Eigen::Vector3d p_left( 0.0,  arm_length, 0.0);   // +Y (LEFT)
  const Eigen::Vector3d p_right(0.0, -arm_length, 0.0);   // -Y (RIGHT)

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

  // Build spatial inertia
  RotationalInertia<double> J(Ixx, Iyy, Izz, 0.0, 0.0, 0.0);
  SpatialInertia<double> M_Bcm = SpatialInertia<double>::MakeFromCentralInertia(
      total_mass, Eigen::Vector3d::Zero(), J);

  const auto& body = plant->AddRigidBody("quadcopter_body", M_Bcm);

  // ===== COLLISION GEOMETRY =====
  const double collision_box_x = arm_length * 2.0;
  const double collision_box_y = arm_length * 2.0;
  const double collision_box_z = body_z;
  plant->RegisterCollisionGeometry(
      body,
      RigidTransformd::Identity(),
      geometry::Box(collision_box_x, collision_box_y, collision_box_z),
      "body_collision",
      multibody::CoulombFriction<double>(0.5, 0.5));

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

  // ===== VISUAL GEOMETRY =====
  using drake::Vector4;
  
  const double body_size = 0.09;
  const double top_plate_z = body_z / 2.0;

  // Central top plate (dark gray)
  plant->RegisterVisualGeometry(
      body, RigidTransformd(Eigen::Vector3d(0, 0, top_plate_z)),
      geometry::Box(body_size, body_size, 0.006),
      "top_plate", Vector4<double>(0.2, 0.2, 0.2, 1.0));

  // Arms with directional color coding
  const double arm_width = 0.02;
  const double arm_length_visual = arm_length;
  
  // Front arm (+X) - BRIGHT BLUE
  plant->RegisterVisualGeometry(
      body, RigidTransformd(Eigen::Vector3d(arm_length_visual/2.0, 0, 0)),
      geometry::Box(arm_length_visual, arm_width, 0.004),
      "arm_front", Vector4<double>(0.2, 0.4, 1.0, 1.0));  // Blue
  
  // Back arm (-X) - GREEN
  plant->RegisterVisualGeometry(
      body, RigidTransformd(Eigen::Vector3d(-arm_length_visual/2.0, 0, 0)),
      geometry::Box(arm_length_visual, arm_width, 0.004),
      "arm_back", Vector4<double>(0.2, 0.8, 0.2, 1.0));  // Green
  
  // Left arm (+Y) - RED
  plant->RegisterVisualGeometry(
      body, RigidTransformd(Eigen::Vector3d(0, arm_length_visual/2.0, 0)),
      geometry::Box(arm_width, arm_length_visual, 0.004),
      "arm_left", Vector4<double>(1.0, 0.2, 0.2, 1.0));  // Red
  
  // Right arm (-Y) - YELLOW
  plant->RegisterVisualGeometry(
      body, RigidTransformd(Eigen::Vector3d(0, -arm_length_visual/2.0, 0)),
      geometry::Box(arm_width, arm_length_visual, 0.004),
      "arm_right", Vector4<double>(1.0, 1.0, 0.2, 1.0));  // Yellow

  // Rotor spheres with matching colors (larger for visibility)
  const double rotor_vis_r = 0.025;
  
  // FRONT rotor (+X) - BRIGHT BLUE
  plant->RegisterVisualGeometry(body, RigidTransformd(p_front),
      geometry::Sphere(rotor_vis_r), "rotor_front", 
      Vector4<double>(0.2, 0.4, 1.0, 1.0));
  
  // BACK rotor (-X) - GREEN
  plant->RegisterVisualGeometry(body, RigidTransformd(p_back),
      geometry::Sphere(rotor_vis_r), "rotor_back", 
      Vector4<double>(0.2, 0.8, 0.2, 1.0));
  
  // LEFT rotor (+Y) - RED
  plant->RegisterVisualGeometry(body, RigidTransformd(p_left),
      geometry::Sphere(rotor_vis_r), "rotor_left", 
      Vector4<double>(1.0, 0.2, 0.2, 1.0));
  
  // RIGHT rotor (-Y) - YELLOW
  plant->RegisterVisualGeometry(body, RigidTransformd(p_right),
      geometry::Sphere(rotor_vis_r), "rotor_right", 
      Vector4<double>(1.0, 1.0, 0.2, 1.0));

  return body;
}

void AddGround(multibody::MultibodyPlant<double>* plant,
               geometry::SceneGraph<double>* scene_graph) {
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
        geometry::Box(line_width, grid_spacing * grid_count * 2, 0.001),
        "grid_y_" + std::to_string(i),
        Vector4<double>(0.15, 0.3, 0.15, 1.0));
  }
}

}  // namespace drake