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

#include "drake/multibody/tree/fixed_offset_frame.h"

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

  // Motor positions (X-CONFIGURATION - motors on diagonals)
  const Eigen::Vector3d p_fr( arm_length, -arm_length, 0.0);  // Blue  - Front-Right
  const Eigen::Vector3d p_fl( arm_length,  arm_length, 0.0);  // Red   - Front-Left
  const Eigen::Vector3d p_br(-arm_length, -arm_length, 0.0);  // Yellow - Back-Right
  const Eigen::Vector3d p_bl(-arm_length,  arm_length, 0.0);  // Green - Back-Left

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

  add_point_mass_inertia(p_fr, m_motor);
  add_point_mass_inertia(p_fl, m_motor);
  add_point_mass_inertia(p_br, m_motor);
  add_point_mass_inertia(p_bl, m_motor);

  // Build spatial inertia
  RotationalInertia<double> J(Ixx, Iyy, Izz, 0.0, 0.0, 0.0);
  SpatialInertia<double> M_Bcm = SpatialInertia<double>::MakeFromCentralInertia(
      total_mass, Eigen::Vector3d::Zero(), J);

  const auto& body = plant->AddRigidBody("quadcopter_body", M_Bcm);

  // ========================================================================
  // ADD FPV CAMERA FRAME
  // ========================================================================
  plant->AddFrame(
      std::make_unique<multibody::FixedOffsetFrame<double>>(
          "fpv_camera",
          body.body_frame(),
          math::RigidTransformd()
      )
  );
  
  const Eigen::Vector3d camera_position(0.05, 0.0, 0.02);
  const Eigen::Vector3d camera_vis_pos(0.08, 0.0, 0.02);
  plant->RegisterVisualGeometry(
      body,
      math::RigidTransformd(camera_vis_pos),
      geometry::Box(0.015, 0.020, 0.008),
      "camera_body",
      drake::Vector4<double>(0.1, 0.1, 0.1, 1.0)
  );
  
  const Eigen::Vector3d lens_pos = camera_position + Eigen::Vector3d(0.012, 0, 0);
  plant->RegisterVisualGeometry(
      body,
      math::RigidTransformd(lens_pos),
      geometry::Sphere(0.004),
      "camera_lens",
      drake::Vector4<double>(0.2, 0.4, 0.9, 1.0)
  );

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
  plant->RegisterCollisionGeometry(body, RigidTransformd(p_fr),
      geometry::Sphere(rotor_collision_r), "rotor_collision_fr",
      multibody::CoulombFriction<double>(0.5, 0.5));
  plant->RegisterCollisionGeometry(body, RigidTransformd(p_fl),
      geometry::Sphere(rotor_collision_r), "rotor_collision_fl",
      multibody::CoulombFriction<double>(0.5, 0.5));
  plant->RegisterCollisionGeometry(body, RigidTransformd(p_br),
      geometry::Sphere(rotor_collision_r), "rotor_collision_br",
      multibody::CoulombFriction<double>(0.5, 0.5));
  plant->RegisterCollisionGeometry(body, RigidTransformd(p_bl),
      geometry::Sphere(rotor_collision_r), "rotor_collision_bl",
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

  // Arms as diagonals (X-configuration)
  // Each arm goes from center to a diagonal corner
  const double arm_width = 0.02;
  const double arm_diag_length = std::sqrt(2.0) * arm_length;  // Diagonal length
  
  // Front-Right arm (center to FR) - BLUE
  plant->RegisterVisualGeometry(
      body, RigidTransformd(
          math::RotationMatrixd::MakeZRotation(-M_PI / 4.0),
          Eigen::Vector3d(arm_length / 2.0, -arm_length / 2.0, 0)),
      geometry::Box(arm_diag_length, arm_width, 0.004),
      "arm_fr", Vector4<double>(0.2, 0.4, 1.0, 1.0));  // Blue
  
  // Front-Left arm (center to FL) - RED
  plant->RegisterVisualGeometry(
      body, RigidTransformd(
          math::RotationMatrixd::MakeZRotation(M_PI / 4.0),
          Eigen::Vector3d(arm_length / 2.0, arm_length / 2.0, 0)),
      geometry::Box(arm_diag_length, arm_width, 0.004),
      "arm_fl", Vector4<double>(1.0, 0.2, 0.2, 1.0));  // Red
  
  // Back-Right arm (center to BR) - YELLOW
  plant->RegisterVisualGeometry(
      body, RigidTransformd(
          math::RotationMatrixd::MakeZRotation(M_PI / 4.0),
          Eigen::Vector3d(-arm_length / 2.0, -arm_length / 2.0, 0)),
      geometry::Box(arm_diag_length, arm_width, 0.004),
      "arm_br", Vector4<double>(1.0, 1.0, 0.2, 1.0));  // Yellow
  
  // Back-Left arm (center to BL) - GREEN
  plant->RegisterVisualGeometry(
      body, RigidTransformd(
          math::RotationMatrixd::MakeZRotation(-M_PI / 4.0),
          Eigen::Vector3d(-arm_length / 2.0, arm_length / 2.0, 0)),
      geometry::Box(arm_diag_length, arm_width, 0.004),
      "arm_bl", Vector4<double>(0.2, 0.8, 0.2, 1.0));  // Green

  // Rotor spheres at diagonal positions
  const double rotor_vis_r = 0.025;
  
  // BLUE - Front-Right
  plant->RegisterVisualGeometry(body, RigidTransformd(p_fr),
      geometry::Sphere(rotor_vis_r), "rotor_fr", 
      Vector4<double>(0.2, 0.4, 1.0, 1.0));
  
  // RED - Front-Left
  plant->RegisterVisualGeometry(body, RigidTransformd(p_fl),
      geometry::Sphere(rotor_vis_r), "rotor_fl", 
      Vector4<double>(1.0, 0.2, 0.2, 1.0));
  
  // YELLOW - Back-Right
  plant->RegisterVisualGeometry(body, RigidTransformd(p_br),
      geometry::Sphere(rotor_vis_r), "rotor_br", 
      Vector4<double>(1.0, 1.0, 0.2, 1.0));
  
  // GREEN - Back-Left
  plant->RegisterVisualGeometry(body, RigidTransformd(p_bl),
      geometry::Sphere(rotor_vis_r), "rotor_bl", 
      Vector4<double>(0.2, 0.8, 0.2, 1.0));

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