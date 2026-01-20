#include "quadcopter_model.h"

#include "drake/geometry/geometry_ids.h"
#include "drake/math/rotation_matrix.h"

namespace drake {

const multibody::RigidBody<double>& AddQuadcopterModel(
    multibody::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph) {
  // Drone properties
  const double body_mass = 0.5; // 500 grams total
  const double body_size = 0.08; // 8cm center body
  const double body_height = 0.04; // 4cm thick center
  const double arm_length = 0.15; // 15cm arms
  const double arm_width = 0.025; // 2.5cm arm width
  const double arm_height = 0.015; // 1.5cm arm thickness
  const double rotor_radius = 0.04; // 4cm rotor radius
  const double rotor_height = 0.008; // 0.8cm rotor thickness

  // Create the main body
  const auto& drone_body = plant->AddRigidBody(
      "quadcopter",
      multibody::SpatialInertia<double>::SolidBoxWithMass(
          body_mass, body_size, body_size, body_height));

  // CENTER BODY (Red/Black)
  plant->RegisterVisualGeometry(
      drone_body,
      math::RigidTransformd::Identity(),
      geometry::Box(body_size, body_size, body_height),
      "center_body",
      Vector4<double>(0.8, 0.1, 0.1, 1.0)); // Dark red

  plant->RegisterVisualGeometry(
      drone_body,
      math::RigidTransformd(Eigen::Vector3d(0, 0, body_height/2 + 0.003)),
      geometry::Box(body_size * 0.9, body_size * 0.9, 0.006),
      "top_plate",
      Vector4<double>(0.1, 0.1, 0.1, 1.0)); // Black

  // CROSS ARMS (4 arms forming a + shape)
  std::vector<Vector4<double>> arm_colors = {
      Vector4<double>(0.3, 0.3, 0.3, 1.0), // Front - dark gray
      Vector4<double>(0.4, 0.4, 0.4, 1.0), // Back - lighter gray
      Vector4<double>(0.3, 0.3, 0.3, 1.0), // Left - dark gray
      Vector4<double>(0.4, 0.4, 0.4, 1.0) // Right - lighter gray
  };

  std::vector<Eigen::Vector3d> arm_positions = {
      {arm_length/2, 0, 0}, // Front arm (along +X)
      {-arm_length/2, 0, 0}, // Back arm (along -X)
      {0, arm_length/2, 0}, // Left arm (along +Y)
      {0, -arm_length/2, 0} // Right arm (along -Y)
  };

  for (size_t i = 0; i < 4; ++i) {
    double length = (i < 2) ? arm_length : arm_width;
    double width = (i < 2) ? arm_width : arm_length;
    plant->RegisterVisualGeometry(
        drone_body,
        math::RigidTransformd(arm_positions[i]),
        geometry::Box(length, width, arm_height),
        "arm_" + std::to_string(i),
        arm_colors[i]);
  }

  // ROTORS (4 propellers at arm ends)
  std::vector<Eigen::Vector3d> rotor_positions = {
      {arm_length, 0, arm_height/2}, // Front rotor
      {-arm_length, 0, arm_height/2}, // Back rotor
      {0, arm_length, arm_height/2}, // Left rotor
      {0, -arm_length, arm_height/2} // Right rotor
  };

  std::vector<Vector4<double>> rotor_colors = {
      Vector4<double>(0.15, 0.15, 0.15, 1.0), // Front - very dark
      Vector4<double>(0.15, 0.15, 0.15, 1.0), // Back - very dark
      Vector4<double>(0.25, 0.25, 0.25, 1.0), // Left - slightly lighter
      Vector4<double>(0.25, 0.25, 0.25, 1.0) // Right - slightly lighter
  };

  for (size_t i = 0; i < rotor_positions.size(); ++i) {
    plant->RegisterVisualGeometry(
        drone_body,
        math::RigidTransformd(rotor_positions[i]),
        geometry::Cylinder(rotor_radius, rotor_height),
        "rotor_" + std::to_string(i),
        rotor_colors[i]);
  }

  // MOTOR MOUNTS (small cylinders connecting arms to rotors)
  const double motor_radius = 0.012; // 1.2cm motor diameter
  const double motor_height = 0.025; // 2.5cm motor height

  for (size_t i = 0; i < rotor_positions.size(); ++i) {
    Eigen::Vector3d motor_pos = rotor_positions[i];
    motor_pos[2] = 0; // Motors sit at arm level
    plant->RegisterVisualGeometry(
        drone_body,
        math::RigidTransformd(motor_pos),
        geometry::Cylinder(motor_radius, motor_height),
        "motor_" + std::to_string(i),
        Vector4<double>(0.2, 0.2, 0.2, 1.0)); // Dark gray motors
  }

  // LANDING GEAR (4 small legs)
  const double leg_radius = 0.006; // 6mm diameter legs
  const double leg_height = 0.06; // 6cm tall legs

  std::vector<Eigen::Vector3d> leg_positions = {
      {body_size/2, body_size/2, -leg_height/2}, // Front-right
      {body_size/2, -body_size/2, -leg_height/2}, // Front-left
      {-body_size/2, body_size/2, -leg_height/2}, // Back-right
      {-body_size/2, -body_size/2, -leg_height/2} // Back-left
  };

  for (size_t i = 0; i < leg_positions.size(); ++i) {
    plant->RegisterVisualGeometry(
        drone_body,
        math::RigidTransformd(leg_positions[i]),
        geometry::Cylinder(leg_radius, leg_height),
        "leg_" + std::to_string(i),
        Vector4<double>(0.15, 0.15, 0.15, 1.0)); // Very dark legs
  }

  // COLLISION GEOMETRY (simplified - just the main body)
  plant->RegisterCollisionGeometry(
      drone_body,
      math::RigidTransformd::Identity(),
      geometry::Box(arm_length * 2, arm_length * 2, body_height),
      "body_collision",
      multibody::CoulombFriction<double>(0.5, 0.5));

  return drone_body;
}

void AddGround(multibody::MultibodyPlant<double>* plant,
               geometry::SceneGraph<double>* scene_graph) {
  plant->RegisterCollisionGeometry(
      plant->world_body(),
      math::RigidTransformd::Identity(),
      geometry::HalfSpace(),
      "ground_collision",
      multibody::CoulombFriction<double>(0.9, 0.9));

  // Main ground plane
  plant->RegisterVisualGeometry(
      plant->world_body(),
      math::RigidTransformd(Eigen::Vector3d(0, 0, -0.005)),
      geometry::Box(30, 30, 0.01),
      "ground_visual",
      Vector4<double>(0.2, 0.4, 0.2, 1.0)); // Green ground

  // Add some grid lines for depth perception
  const int grid_count = 10;
  const double grid_spacing = 2.0;
  const double line_width = 0.02;

  for (int i = -grid_count; i <= grid_count; ++i) {
    // Lines along X axis
    plant->RegisterVisualGeometry(
        plant->world_body(),
        math::RigidTransformd(Eigen::Vector3d(0, i * grid_spacing, -0.004)),
        geometry::Box(grid_spacing * grid_count * 2, line_width, 0.001),
        "grid_x_" + std::to_string(i),
        Vector4<double>(0.15, 0.3, 0.15, 1.0));

    // Lines along Y axis
    plant->RegisterVisualGeometry(
        plant->world_body(),
        math::RigidTransformd(Eigen::Vector3d(i * grid_spacing, 0, -0.004)),
        geometry::Box(line_width, grid_spacing * grid_count * 2, 0.001),
        "grid_y_" + std::to_string(i),
        Vector4<double>(0.15, 0.3, 0.15, 1.0));
  }
}

}  // namespace drake
