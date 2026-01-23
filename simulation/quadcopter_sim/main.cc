// main.cc - Angle Stabilization Mode (Auto-Level)
#include <iostream>
#include <memory>
#include <algorithm>
#include <chrono>
#include <thread>
#include <limits>
#include <iomanip>

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
#include "drake/common/value.h"
#include "drake/systems/framework/basic_vector.h"

#include "keyboard_input.h"
#include "quadcopter_controller.h"
#include "quadcopter_model.h"
#include "world.h"

using namespace drake;

int main() {
  std::cout << "Starting quadcopter simulation - ANGLE STABILIZATION MODE..." << std::endl;

  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);

  const auto& drone_body = AddQuadcopterModel(&plant, &scene_graph);
  AddGroundWithCollision(&plant, &scene_graph, /*ground_z=*/-0.01);

  plant.Finalize();

  auto controller = builder.AddSystem<systems::QuadcopterController>(
      &plant, &drone_body);
  controller->set_name("cascaded_angle_rate_controller");

  // Export control input port (port 0)
  builder.ExportInput(controller->get_input_port(0), "control_input");
  
  // Connect plant state to controller (port 1)
  builder.Connect(plant.get_state_output_port(),
                  controller->get_input_port(1));
  
  // Connect controller output to plant
  builder.Connect(controller->get_output_port(0),
                  plant.get_applied_spatial_force_input_port());

  // Create MeshCat and connect visualizer
  auto meshcat = std::make_shared<geometry::Meshcat>();
  auto meshcat_vis = &geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat);
  meshcat_vis->set_name("meshcat_visualizer");
  
  std::cout << "\nMeshCat URL: " << meshcat->web_url() << std::endl;

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  auto& root_context = simulator.get_mutable_context();
  auto& plant_context = plant.GetMyMutableContextFromRoot(&root_context);

  // Start pose at ground level (z=0)
  math::RollPitchYaw<double> rpy(0.0, 0.0, 0.0);
  math::RigidTransformd initial_pose(
      rpy.ToRotationMatrix(),
      Eigen::Vector3d(0, 0, 0.0));  // Start at ground
  plant.SetFreeBodyPose(&plant_context, drone_body, initial_pose);
  plant.SetFreeBodySpatialVelocity(&plant_context, drone_body,
                                   multibody::SpatialVelocity<double>::Zero());

  diagram->ForcedPublish(root_context);

  const double body_mass = 0.5;
  const double hover_thrust = body_mass * 9.81;  // Total thrust needed to hover

  std::cout << "\n╔══════════════════════════════════════════════════╗" << std::endl;
  std::cout << "║   QUADCOPTER ANGLE STABILIZATION (AUTO-LEVEL)  ║" << std::endl;
  std::cout << "╚══════════════════════════════════════════════════╝" << std::endl;
  std::cout << "\nDrone specifications:" << std::endl;
  std::cout << " • Mass: " << body_mass << " kg" << std::endl;
  std::cout << " • Hover thrust: " << hover_thrust << " N" << std::endl;

  std::cout << "\n═══════════════════════════════════════════════════" << std::endl;
  std::cout << "CONTROLS - STABILIZE MODE (Auto-Level):" << std::endl;
  std::cout << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << "\nTHRUST:" << std::endl;
  std::cout << " • Arrow Up / Down    : Increase/Decrease total thrust" << std::endl;
  std::cout << "\nATTITUDE (hold to tilt, release to AUTO-LEVEL):" << std::endl;
  std::cout << " • i / k              : Pitch forward/back" << std::endl;
  std::cout << " • j / l              : Roll left/right" << std::endl;
  std::cout << " • u / o              : Yaw left/right" << std::endl;
  std::cout << "\nSYSTEM:" << std::endl;
  std::cout << " • a                  : Arm/Disarm" << std::endl;
  std::cout << " • q                  : Quit" << std::endl;
  std::cout << "\n═══════════════════════════════════════════════════" << std::endl;
  std::cout << "\n✨ AUTO-STABILIZATION BEHAVIOR:" << std::endl;
  std::cout << " • Hold key → Drone tilts → Moves in that direction" << std::endl;
  std::cout << " • Release key → Drone AUTO-LEVELS → Movement STOPS" << std::endl;
  std::cout << " • No need to counter-steer - it levels itself!" << std::endl;
  std::cout << " • This is like DJI/consumer drones - EASY to fly!" << std::endl;
  std::cout << "═══════════════════════════════════════════════════\n" << std::endl;

  std::cout << "MeshCat URL: " << meshcat->web_url() << std::endl;
  std::cout << "\nPress Enter to start..." << std::endl;
  std::cin.get();

  drake::set_conio_terminal_mode();
  simulator.set_target_realtime_rate(1.0);

  double sim_time = 0.0;
  const double dt = 0.005;
  const double end_time = 300.0;

  // Control state
  double thrust = 0.0;  // Start with ZERO thrust (drone on ground)
  const double thrust_inc = 0.5;  // Thrust increment (N)
  
  // Angle setpoints (rad) - drone will auto-level to these when keys released
  // When no key pressed, these decay to zero → drone levels out
  double target_roll = 0.0;
  double target_pitch = 0.0;
  double target_yaw = 0.0;
  
  const double max_angle = 0.35;  // Maximum tilt ~20 degrees (slightly higher for responsiveness)
  const double angle_inc = 0.04;  // Angle increment per key press (more responsive)
  
  bool armed = false;
  bool running = true;

  // Control input vector: 4 elements [thrust, roll_angle, pitch_angle, yaw_angle]
  Eigen::Matrix<double, 4, 1> control_input;
  control_input.setZero();
  
  // Track time for periodic status updates
  double last_print_time = 0.0;

  while (running && sim_time < end_time) {
    int key = drake::get_key();

    // ========================================================================
    // ARM / DISARM
    // ========================================================================
    if (key == 'a' || key == 'A') {
      armed = !armed;
      if (armed) {
        sim_time = 0.0;
        root_context.SetTime(0.0);
        std::cout << "\n*** ARMED - Motors active ***" << std::endl;
      } else {
        std::cout << "\n*** DISARMED - Motors off ***" << std::endl;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }

    // ========================================================================
    // QUIT
    // ========================================================================
    if (key == 'q' || key == 'Q') {
      std::cout << "\nQuit requested." << std::endl;
      break;
    }

    // ========================================================================
    // THRUST CONTROL (Arrow keys)
    // ========================================================================
    if (key == drake::KEY_ARROW_UP) {
      thrust += thrust_inc;
      thrust = std::min(thrust, hover_thrust * 2.0);  // Max 2x hover thrust
      std::cout << "Thrust: " << thrust << " N" << std::endl;
    } else if (key == drake::KEY_ARROW_DOWN) {
      thrust -= thrust_inc;
      thrust = std::max(thrust, 0.0);
      std::cout << "Thrust: " << thrust << " N" << std::endl;
    }

    // ========================================================================
    // ANGLE CONTROL (i/k/j/l/u/o keys)
    // KEY BEHAVIOR: Angles decay toward zero when no key pressed
    // This creates auto-leveling: release key → angle → 0 → drone levels
    // ========================================================================
    
    // AGGRESSIVE auto-decay toward zero for FAST stopping
    // Higher decay = faster return to level = quicker stop
    const double decay_factor = 0.80;  // 20% decay per frame → levels in ~0.3s (was 0.92)
    target_roll *= decay_factor;
    target_pitch *= decay_factor;
    // Note: yaw doesn't decay - holds heading
    
    // Apply key inputs (incremental, builds up while held)
    if (key == 'i' || key == 'I') {
      // Pitch forward
      target_pitch += angle_inc;
      target_pitch = std::min(target_pitch, max_angle);
    } else if (key == 'k' || key == 'K') {
      // Pitch backward
      target_pitch -= angle_inc;
      target_pitch = std::max(target_pitch, -max_angle);
    }
    
    if (key == 'j' || key == 'J') {
      // Roll left
      target_roll -= angle_inc;
      target_roll = std::max(target_roll, -max_angle);
    } else if (key == 'l' || key == 'L') {
      // Roll right
      target_roll += angle_inc;
      target_roll = std::min(target_roll, max_angle);
    }
    
    if (key == 'u' || key == 'U') {
      // Yaw left
      target_yaw += angle_inc;
    } else if (key == 'o' || key == 'O') {
      // Yaw right
      target_yaw -= angle_inc;
    }
    
    // Normalize yaw to [-π, π]
    while (target_yaw > M_PI) target_yaw -= 2.0 * M_PI;
    while (target_yaw < -M_PI) target_yaw += 2.0 * M_PI;
    
    // Zero out very small angles (dead zone for cleaner behavior)
    if (std::abs(target_roll) < 0.005) target_roll = 0.0;
    if (std::abs(target_pitch) < 0.005) target_pitch = 0.0;

    // ========================================================================
    // BUILD CONTROL INPUT
    // ========================================================================
    if (!armed) {
      control_input.setZero();
    } else {
      control_input(0) = thrust;
      control_input(1) = target_roll;    // Target angles (not rates!)
      control_input(2) = target_pitch;
      control_input(3) = target_yaw;
    }

    // Send to controller
    root_context.FixInputPort(
        0,
        drake::Value<drake::systems::BasicVector<double>>(
            drake::systems::BasicVector<double>(control_input)));

    // ========================================================================
    // ADVANCE SIMULATION
    // ========================================================================
    if (armed) {
      simulator.AdvanceTo(sim_time + dt);
      sim_time += dt;
      
      // Print status every 1 second
      if (sim_time - last_print_time >= 1.0) {
        auto& current_plant_context = plant.GetMyMutableContextFromRoot(&root_context);
        const math::RigidTransformd current_pose = 
            plant.GetFreeBodyPose(current_plant_context, drone_body);
        const Eigen::Vector3d pos = current_pose.translation();
        const math::RollPitchYaw<double> current_rpy(current_pose.rotation());
        
        std::cout << "t=" << std::fixed << std::setprecision(1) << sim_time 
                  << " | Thrust=" << std::setprecision(1) << thrust 
                  << "N | Pos=[" << std::setprecision(2) 
                  << pos(0) << ", " << pos(1) << ", " << pos(2) << "]m"
                  << " | Target=[" << std::setprecision(1)
                  << target_roll*57.3 << "°, " 
                  << target_pitch*57.3 << "°, " 
                  << target_yaw*57.3 << "°]"
                  << " | Actual=[" << std::setprecision(1)
                  << current_rpy.roll_angle()*57.3 << "°, " 
                  << current_rpy.pitch_angle()*57.3 << "°, " 
                  << current_rpy.yaw_angle()*57.3 << "°]" << std::endl;
        last_print_time = sim_time;
      }
    } else {
      diagram->ForcedPublish(root_context);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  std::cout << "\nSimulation stopped." << std::endl;
  
  // Restore terminal to canonical mode so cin.get() works
  struct termios term;
  tcgetattr(STDIN_FILENO, &term);
  term.c_lflag |= ICANON;  // Enable canonical mode
  term.c_lflag |= ECHO;    // Enable echo
  tcsetattr(STDIN_FILENO, TCSANOW, &term);
  
  std::cout << "Press Enter to exit..." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  std::cin.get();
  return 0;
}