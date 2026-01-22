#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/geometry/scene_graph.h"

namespace drake {

// Add a collision-enabled ground and visual grid to the plant/scene_graph.
//
// Args:
//  - plant: the MultibodyPlant to register collision & visuals with.
//  - scene_graph: the SceneGraph used for visualization (may be unused here).
//  - ground_z: the z coordinate of the ground plane (HalfSpace is placed at this z).
//              Default -0.01 places the plane 1 cm below origin.
void AddGroundWithCollision(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    double ground_z = -0.01);

}  // namespace drake
