#ifndef QUADCOPTER_MODEL_H_
#define QUADCOPTER_MODEL_H_

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {

const multibody::RigidBody<double>& AddQuadcopterModel(
    multibody::MultibodyPlant<double>* plant,
    geometry::SceneGraph<double>* scene_graph);

void AddGround(multibody::MultibodyPlant<double>* plant,
               geometry::SceneGraph<double>* scene_graph);

}  // namespace drake

#endif  // QUADCOPTER_MODEL_H_
