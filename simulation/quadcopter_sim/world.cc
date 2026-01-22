#include "world.h"

#include "drake/math/rigid_transform.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/geometry/shape_specification.h"

using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using drake::Vector4;

namespace drake {

void AddGroundWithCollision(
    MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* /*scene_graph*/,
    double ground_z) {
  // Register collision geometry (HalfSpace) on the world so bodies collide with ground.
  // Place the half space so the top plane is at z = ground_z.
  plant->RegisterCollisionGeometry(
      plant->world_body(),
      RigidTransformd(Eigen::Vector3d(0, 0, ground_z)),
      geometry::HalfSpace(),
      "ground_collision",
      multibody::CoulombFriction<double>(0.9, 0.9));

  // Visual ground plane (thin box) slightly below the halfspace plane for rendering.
  const double ground_thickness = 0.01;
  const double ground_half_extents = 30.0;
  plant->RegisterVisualGeometry(
      plant->world_body(),
      RigidTransformd(Eigen::Vector3d(0, 0, ground_z - ground_thickness / 2.0)),
      geometry::Box(ground_half_extents, ground_half_extents, ground_thickness),
      "ground_visual",
      Vector4<double>(0.2, 0.4, 0.2, 1.0));  // greenish

  // Optional: grid lines for depth perception (visual only)
  const int grid_count = 10;
  const double grid_spacing = 2.0;
  const double line_width = 0.02;
  for (int i = -grid_count; i <= grid_count; ++i) {
    plant->RegisterVisualGeometry(
        plant->world_body(),
        RigidTransformd(Eigen::Vector3d(0, i * grid_spacing, ground_z + 0.0005)),
        geometry::Box(grid_spacing * grid_count * 2, line_width, 0.001),
        "grid_x_" + std::to_string(i),
        Vector4<double>(0.15, 0.3, 0.15, 1.0));
    plant->RegisterVisualGeometry(
        plant->world_body(),
        RigidTransformd(Eigen::Vector3d(i * grid_spacing, 0, ground_z + 0.0005)),
        geometry::Box(line_width, grid_spacing * grid_count * 2, 0.001),
        "grid_y_" + std::to_string(i),
        Vector4<double>(0.15, 0.3, 0.15, 1.0));
  }
}

}  // namespace drake
