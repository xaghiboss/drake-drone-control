// main.cc
#include <iostream>
#include <memory>
#include <algorithm>
#include <chrono>
#include <thread>
#include <limits>

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
  std::cout << "Starting quadcopter simulation with PID attitude control..." << std::endl;

  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);

  const auto& drone_body = AddQuadcopterModel(&plant, &scene_graph);
  AddGroundWithCollision(&plant, &scene_graph, /*ground_z=*/-0.01);

  plant.Finalize();

  auto controller = builder.AddSystem<systems::QuadcopterController>(
      &plant, &drone_body);
  controller->set_name("pid_attitude_controller");

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
  std::cout << "║  QUADCOPTER PID ATTITUDE CONTROL SIMULATION  ║" << std::endl;
  std::cout << "╚══════════════════════════════════════════════════╝" << std::endl;
  std::cout << "\nDrone specifications:" << std::endl;
  std::cout << " • Mass: " << body_mass << " kg" << std::endl;
  std::cout << " • Hover thrust: " << hover_thrust << " N" << std::endl;

  std::cout << "\nControls (CLOSED-LOOP PID):" << std::endl;
  std::cout << " • Arrow Up / Down    : Increase/Decrease thrust (altitude)" << std::endl;
  std::cout << " • i / k              : Pitch forward/back (SETPOINT)" << std::endl;
  std::cout << " • j / l              : Roll left/right (SETPOINT)" << std::endl;
  std::cout << " • u / o              : Yaw left/right (SETPOINT)" << std::endl;
  std::cout << " • SPACE              : Reset all attitude setpoints to 0" << std::endl;
  std::cout << " • a                  : Arm/Disarm" << std::endl;
  std::cout << " • q                  : Quit" << std::endl;
  std::cout << "\nNOTE: Attitude setpoints auto-stabilize (PID control)" << std::endl;

  std::cout << "\nMeshCat URL: " << meshcat->web_url() << std::endl;
  std::cout << "\nPress Enter to start..." << std::endl;
  std::cin.get();

  drake::set_conio_terminal_mode();
  simulator.set_target_realtime_rate(1.0);

  double sim_time = 0.0;
  const double dt = 0.005;
  const double end_time = 300.0;

  // Control state
  double thrust = 0.0;  // Start with ZERO thrust (drone on ground)
  const double thrust_inc = 0.5;       // Thrust increment (N)
  
  // Attitude setpoints (rad)
  double desired_roll = 0.0;
  double desired_pitch = 0.0;
  double desired_yaw = 0.0;
  
  const double angle_inc = 0.05;  // 2.86 degrees per keypress
  const double max_angle = 0.5;   // Max tilt ~28.6 degrees
  
  bool armed = false;
  bool running = true;

  // Control input vector: 10 elements
  // [0] total_thrust, [1] roll, [2] pitch, [3] yaw, [4-6] rates, [7-9] reserved
  Eigen::Matrix<double, 10, 1> control_input;
  control_input.setZero();

  while (running && sim_time < end_time) {
    int key = drake::get_key();

    // Arm / disarm
    if (key == 'a' || key == 'A') {
      armed = !armed;
      if (armed) {
        sim_time = 0.0;
        root_context.SetTime(0.0);
        std::cout << "*** ARMED ***" << std::endl;
      } else {
        std::cout << "*** DISARMED ***" << std::endl;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }

    // Quit
    if (key == 'q' || key == 'Q') {
      std::cout << "Quit requested." << std::endl;
      break;
    }

    // Reset all attitude setpoints
    if (key == ' ') {
      desired_roll = 0.0;
      desired_pitch = 0.0;
      desired_yaw = 0.0;
      std::cout << "Attitude setpoints reset to level" << std::endl;
    }

    // Thrust control
    if (key == drake::KEY_ARROW_UP) {
      thrust += thrust_inc;
      thrust = std::min(thrust, hover_thrust * 2.0);  // Max 2x hover
    } else if (key == drake::KEY_ARROW_DOWN) {
      thrust -= thrust_inc;
      thrust = std::max(thrust, 0.0);
    }

    // Pitch setpoint control
    if (key == 'i' || key == 'I') {
      desired_pitch += angle_inc;  // Pitch forward (positive)
      desired_pitch = std::min(desired_pitch, max_angle);
    } else if (key == 'k' || key == 'K') {
      desired_pitch -= angle_inc;  // Pitch back (negative)
      desired_pitch = std::max(desired_pitch, -max_angle);
    }

    // Roll setpoint control
    if (key == 'j' || key == 'J') {
      desired_roll -= angle_inc;  // Roll left (negative)
      desired_roll = std::max(desired_roll, -max_angle);
    } else if (key == 'l' || key == 'L') {
      desired_roll += angle_inc;  // Roll right (positive)
      desired_roll = std::min(desired_roll, max_angle);
    }

    // Yaw setpoint control
    if (key == 'u' || key == 'U') {
      desired_yaw += angle_inc;  // Yaw left
    } else if (key == 'o' || key == 'O') {
      desired_yaw -= angle_inc;  // Yaw right
    }

    // Normalize yaw to [-pi, pi]
    while (desired_yaw > M_PI) desired_yaw -= 2.0 * M_PI;
    while (desired_yaw < -M_PI) desired_yaw += 2.0 * M_PI;

    // Build control input
    if (!armed) {
      control_input.setZero();
    } else {
      control_input(0) = thrust;
      control_input(1) = desired_roll;
      control_input(2) = desired_pitch;
      control_input(3) = desired_yaw;
      control_input(4) = 0.0;  // desired roll rate
      control_input(5) = 0.0;  // desired pitch rate
      control_input(6) = 0.0;  // desired yaw rate
      control_input(7) = 0.0;  // reserved
      control_input(8) = 0.0;  // reserved
      control_input(9) = 0.0;  // reserved
    }

    // Send to controller
    root_context.FixInputPort(
        0,
        drake::Value<drake::systems::BasicVector<double>>(
            drake::systems::BasicVector<double>(control_input)));

    // Advance simulation
    if (armed) {
      simulator.AdvanceTo(sim_time + dt);
      sim_time += dt;
      
      // Print status every 1 second
      if (static_cast<int>(sim_time * 10) % 10 == 0) {
        auto& current_plant_context = plant.GetMyMutableContextFromRoot(&root_context);
        const math::RigidTransformd current_pose = 
            plant.GetFreeBodyPose(current_plant_context, drone_body);
        const Eigen::Vector3d pos = current_pose.translation();
        const math::RollPitchYaw<double> current_rpy(current_pose.rotation());
        
        std::cout << "t=" << sim_time 
                  << " | Thrust=" << thrust 
                  << " | Pos: [" << pos(0) << ", " << pos(1) << ", " << pos(2) << "]"
                  << " | RPY: [" << current_rpy.roll_angle() 
                  << ", " << current_rpy.pitch_angle() 
                  << ", " << current_rpy.yaw_angle() << "]" << std::endl;
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
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');  // Clear any pending input
  std::cin.get();
  return 0;
}