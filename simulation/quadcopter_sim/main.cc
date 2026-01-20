// main.cc
#include <iostream>
#include <memory>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/time.h>

#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/common/value.h"  // drake::Value<T>
#include "drake/systems/framework/basic_vector.h"

#include "keyboard_input.h"  // For non-blocking input (implemented in keyboard_input.cc)
#include "quadcopter_controller.h"
#include "quadcopter_model.h"

using namespace drake;  // optional; keeps code concise (you can fully qualify if preferred)

// main does not define keyboard helpers any more; they live in keyboard_input.cc
int main() {
  std::cout << "Starting quadcopter simulation..." << std::endl;

  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);

  const auto& drone_body = AddQuadcopterModel(&plant, &scene_graph);
  AddGround(&plant, &scene_graph);

  plant.Finalize();

    auto controller = builder.AddSystem<systems::QuadcopterController>(
      &plant, &drone_body);
  controller->set_name("thrust_controller");

  // Export the controller's input port as a diagram input so we can
  // Fix the diagram input port at runtime (index 0).
  builder.ExportInput(controller->get_input_port(0), "thrust_input");


  // Connect controller spatial forces to the plant.
  builder.Connect(controller->get_output_port(0),
                  plant.get_applied_spatial_force_input_port());

  auto meshcat = std::make_shared<geometry::Meshcat>();
  geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat);

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  auto& root_context = simulator.get_mutable_context();
  auto& plant_context = plant.GetMyMutableContextFromRoot(&root_context);

  // Start pose at (0,0,0.5)
  math::RollPitchYaw<double> rpy(0.0, 0.0, 0.0);
  math::RigidTransformd initial_pose(
      rpy.ToRotationMatrix(),
      Eigen::Vector3d(0, 0, 0.5));
  plant.SetFreeBodyPose(&plant_context, drone_body, initial_pose);
  plant.SetFreeBodySpatialVelocity(&plant_context, drone_body,
                                   multibody::SpatialVelocity<double>::Zero());

  diagram->ForcedPublish(root_context);

  const double body_mass = 0.5;
  const double hover_thrust = (body_mass * 9.81) / 4.0;
  std::cout << "\n╔══════════════════════════════════════════╗" << std::endl;
  std::cout << "║ QUADCOPTER SIMULATION - CROSS CONFIG ║" << std::endl;
  std::cout << "╚══════════════════════════════════════════╝" << std::endl;
  std::cout << "\nDrone specifications:" << std::endl;
  std::cout << " • Mass: " << body_mass << " kg" << std::endl;
  std::cout << " • Configuration: Cross (+) with 4 rotors" << std::endl;
  std::cout << " • Arm length: 0.15 m" << std::endl;
  std::cout << "\nControls:" << std::endl;
  std::cout << " • Arrow Up: Increase thrust (hold to ramp up)" << std::endl;
  std::cout << " • Arrow Down: Decrease thrust (hold to ramp down)" << std::endl;
  std::cout << " • Hover thrust/rotor: " << hover_thrust << " N" << std::endl;
  std::cout << "\nMeshCat URL: " << meshcat->web_url() << std::endl;
  std::cout << "\nPress Enter to start simulation..." << std::endl;
  std::cin.get();

  drake::set_conio_terminal_mode();  // Enable non-blocking keyboard

  simulator.set_target_realtime_rate(1.0);

  double sim_time = 0.0;
  const double end_time = 30.0;  // Run for 30s or adjust
  const double dt = 0.01;  // 10ms steps for responsive input
  double thrust_multiplier = 0.0;  // 0 to 2x hover
  const double thrust_increment = 0.02;  // Small step per key event (adjust for sensitivity)
  Eigen::Vector4d thrust_commands = Eigen::Vector4d::Zero();

  while (sim_time < end_time) {
    int key = drake::get_key();
    if (key == drake::KEY_ARROW_UP) {
      thrust_multiplier += thrust_increment;
      if (thrust_multiplier > 2.0) thrust_multiplier = 2.0;
    } else if (key == drake::KEY_ARROW_DOWN) {
      thrust_multiplier -= thrust_increment;
      if (thrust_multiplier < 0.0) thrust_multiplier = 0.0;
    }

    // Apply uniform thrust to all rotors for vertical control
    thrust_commands.fill(hover_thrust * thrust_multiplier);

    // Fix the diagram's input port (thrust to controller).
    // Context::FixInputPort expects an AbstractValue; drake::Value<Eigen::Vector4d>
    // derives from AbstractValue and is appropriate for a basic vector input.
    root_context.FixInputPort( 0, drake::Value<drake::systems::BasicVector<double>>( drake::systems::BasicVector<double>(thrust_commands)));

    simulator.AdvanceTo(sim_time + dt);
    sim_time += dt;
  }

  std::cout << "\n✓ Simulation complete!" << std::endl;
  std::cout << "Press Enter to exit..." << std::endl;
  std::cin.get();

  return 0;
}
