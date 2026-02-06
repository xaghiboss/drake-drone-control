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

void AddGroundWithCollision(multibody::MultibodyPlant<double>* plant,
                           geometry::SceneGraph<double>* scene_graph,
                           double ground_z) {
  
  const Eigen::Vector4d ground_color(0.3, 0.6, 0.3, 1.0);  // Green
  const Eigen::Vector4d grid_color(0.5, 0.5, 0.5, 1.0);    // Gray
  
  // Ground collision (half-space)
  plant->RegisterCollisionGeometry(
      plant->world_body(),
      math::RigidTransformd(Eigen::Vector3d(0, 0, ground_z)),
      geometry::HalfSpace(),
      "ground_collision",
      geometry::ProximityProperties());
  
  // Ground visual (large box so camera can see it)
  plant->RegisterVisualGeometry(
      plant->world_body(),
      math::RigidTransformd(Eigen::Vector3d(0, 0, ground_z - 0.5)),
      geometry::Box(20.0, 20.0, 1.0),  // 20m x 20m x 1m thick
      "ground_visual",
      ground_color);
  
  // Grid lines (optional, helps with depth perception)
  for (int i = -10; i <= 10; ++i) {
    // Lines along X
    plant->RegisterVisualGeometry(
        plant->world_body(),
        math::RigidTransformd(Eigen::Vector3d(0, i, ground_z + 0.01)),
        geometry::Box(20.0, 0.02, 0.01),
        "grid_x_" + std::to_string(i),
        grid_color);
    
    // Lines along Y
    plant->RegisterVisualGeometry(
        plant->world_body(),
        math::RigidTransformd(Eigen::Vector3d(i, 0, ground_z + 0.01)),
        geometry::Box(0.02, 20.0, 0.01),
        "grid_y_" + std::to_string(i),
        grid_color);
  }
}

}  // namespace drake
