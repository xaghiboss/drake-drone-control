// main.cc - IMU-Based Angle Stabilization Mode
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
#include "drake/geometry/render_vtk/factory.h"

#include "drake/systems/sensors/rgbd_sensor.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "camera_viewer.h"
#include "drake/systems/sensors/camera_info.h"

#include "keyboard_input.h"
#include "quadcopter_controller.h"
#include "quadcopter_model.h"
#include "world.h"
#include "imu_sensor.h"  // NEW: Add IMU sensor header
#include "ekf_estimator.h"

using namespace drake;

int main() {
  std::cout << "Starting quadcopter simulation - IMU-BASED STABILIZATION MODE..." << std::endl;

  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);

  // ========================================================================
  // ADD RENDERER (do this BEFORE adding models)
  // ========================================================================
  geometry::RenderEngineVtkParams params;
  params.default_clear_color = Eigen::Vector3d(0.53, 0.81, 0.92);  // Sky blue
  scene_graph.AddRenderer("renderer", geometry::MakeRenderEngineVtk(params));
  std::cout << "Renderer added to SceneGraph" << std::endl;

  // ========================================================================
  // ADD MODELS
  // ========================================================================
  const auto& drone_body = AddQuadcopterModel(&plant, &scene_graph);
  AddGroundWithCollision(&plant, &scene_graph, /*ground_z=*/-0.01);

  plant.Finalize();

  // ========================================================================
  // ADD FPV CAMERA (attached to drone body, facing forward and down)
  // ========================================================================
  
  std::cout << "Setting up FPV camera..." << std::endl;
  
  // Get drone body frame ID
  const geometry::FrameId drone_frame_id = 
      plant.GetBodyFrameIdOrThrow(drone_body.index());
  
  // Camera position and orientation (in body frame)
  const Eigen::Vector3d camera_position(0.03, -0.03, 0.06);
  const double camera_roll = 0.0 * M_PI / 180.0;
  const double camera_pitch = -135.0 * M_PI / 180.0;
  const double camera_yaw = 135.0 * M_PI / 180.0;
  
  const math::RigidTransformd X_BC(
      math::RollPitchYaw<double>(camera_roll, camera_pitch, camera_yaw),
      camera_position
  );

  // Camera settings
  const int width = 480;
  const int height = 640;
  const double fov = 90.0 * M_PI / 180.0;
  
  // Create depth camera
  geometry::render::DepthRenderCamera depth_camera(
      {"renderer", {width, height, fov}, {0.01, 100.0}, {}},
      {0.01, 100.0}
  );
  
  // Create RgbdSensor
  auto* camera = builder.AddSystem<systems::sensors::RgbdSensor>(
      drone_frame_id,
      X_BC,
      depth_camera
  );
  camera->set_name("fpv_camera");
  
  // Connect to scene graph
  builder.Connect(
      scene_graph.get_query_output_port(),
      camera->query_object_input_port()
  );
  
  // Add viewer
  auto* viewer = builder.AddSystem<systems::CameraViewer>("FPV Camera", 30.0);
  builder.Connect(
      camera->color_image_output_port(),
      viewer->get_input_port(0)
  );
  
  std::cout << "FPV camera ready!" << std::endl;

  // ========================================================================
  // ADD IMU SENSOR (CRITICAL FOR HARDWARE TRANSITION!)
  // ========================================================================
  
  std::cout << "Setting up IMU sensor..." << std::endl;
  
  auto* imu = builder.AddSystem<systems::ImuSensor>(&plant, &drone_body, 200.0);
  imu->set_name("imu_sensor");
  
  // Connect plant state to IMU
  builder.Connect(plant.get_state_output_port(),
                  imu->get_input_port(0));
  
  std::cout << "IMU sensor ready (200 Hz update rate)!" << std::endl;

  // ========================================================================
  // ADD EKF STATE ESTIMATOR (NEW!)
  // ========================================================================
  
  std::cout << "Setting up EKF state estimator..." << std::endl;
  
  auto* ekf = builder.AddSystem<systems::EkfEstimator>(200.0);
  ekf->set_name("ekf_estimator");
  
  // Connect IMU measurements to EKF
  builder.Connect(imu->get_output_port(0),
                  ekf->get_input_port(0));
  
  std::cout << "EKF estimator ready (200 Hz update rate)!" << std::endl;

  // ========================================================================
  // ADD CONTROLLER (NOW READS FROM IMU, NOT PLANT!)
  // ========================================================================
  
  auto controller = builder.AddSystem<systems::QuadcopterController>(
      &plant, &drone_body);
  controller->set_name("cascaded_angle_rate_controller");

  // Export control input port (port 0) - unchanged
  builder.ExportInput(controller->get_input_port(0), "control_input");
  
  // ========================================================================
  // CRITICAL CHANGE: Controller reads from EKF, not raw IMU!
  // ========================================================================
  // OLD:
  // builder.Connect(imu->get_output_port(0),
  //                 controller->get_input_port(1));
  
  // NEW:
  builder.Connect(ekf->get_output_port(0),
                  controller->get_input_port(1));
  
  // Connect controller output to plant - unchanged
  builder.Connect(controller->get_output_port(0),
                  plant.get_applied_spatial_force_input_port());

  // ========================================================================
  // ADD MESHCAT VISUALIZER
  // ========================================================================
  
  auto meshcat = std::make_shared<geometry::Meshcat>();
  auto meshcat_vis = &geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, scene_graph, meshcat);
  meshcat_vis->set_name("meshcat_visualizer");
  
  std::cout << "\nMeshCat URL: " << meshcat->web_url() << std::endl;

  // ========================================================================
  // BUILD DIAGRAM
  // ========================================================================
  
  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  auto& root_context = simulator.get_mutable_context();
  auto& plant_context = plant.GetMyMutableContextFromRoot(&root_context);

  // Set initial conditions
  math::RollPitchYaw<double> rpy(0.0, 0.0, 0.0);
  math::RigidTransformd initial_pose(
      rpy.ToRotationMatrix(),
      Eigen::Vector3d(0, 0, 0.0));  // Start at ground
  plant.SetFreeBodyPose(&plant_context, drone_body, initial_pose);
  plant.SetFreeBodySpatialVelocity(&plant_context, drone_body,
                                   multibody::SpatialVelocity<double>::Zero());

  diagram->ForcedPublish(root_context);

  const double body_mass = 0.5;
  const double hover_thrust = body_mass * 9.81;

  // ========================================================================
  // PRINT STARTUP INFO
  // ========================================================================
  
  std::cout << "\n╔══════════════════════════════════════════════════╗" << std::endl;
  std::cout << "║   QUADCOPTER IMU-BASED STABILIZATION MODE    ║" << std::endl;
  std::cout << "╚══════════════════════════════════════════════════╝" << std::endl;
  std::cout << "\nDrone specifications:" << std::endl;
  std::cout << " • Mass: " << body_mass << " kg" << std::endl;
  std::cout << " • Hover thrust: " << hover_thrust << " N" << std::endl;
  std::cout << " • Configuration: X-FRAME (45° rotated)" << std::endl;
  std::cout << " • IMU Update Rate: 200 Hz" << std::endl;
  std::cout << " • Controller: Reads from IMU (NOT plant state!)" << std::endl;
  
  std::cout << "\n     MOTOR LAYOUT (X-Configuration):" << std::endl;
  std::cout << "           Red (FL)" << std::endl;
  std::cout << "          /        Blue (FR)" << std::endl;
  std::cout << "         /        /" << std::endl;
  std::cout << "        /        /" << std::endl;
  std::cout << "       +--------+" << std::endl;
  std::cout << "        \\        \\" << std::endl;
  std::cout << "         \\        \\" << std::endl;
  std::cout << "          \\        Yellow (BR)" << std::endl;
  std::cout << "           Green (BL)" << std::endl;
  
  std::cout << "\n     MOTOR MIXING:" << std::endl;
  std::cout << "     Forward:  Blue+Red ↑,  Yellow+Green ↓" << std::endl;
  std::cout << "     Backward: Blue+Red ↓,  Yellow+Green ↑" << std::endl;
  std::cout << "     Left:     Blue+Yellow ↑,  Red+Green ↓" << std::endl;
  std::cout << "     Right:    Blue+Yellow ↓,  Red+Green ↑" << std::endl;

  std::cout << "\n═══════════════════════════════════════════════════" << std::endl;
  std::cout << "CONTROLS - STABILIZE MODE (Auto-Level):" << std::endl;
  std::cout << "═══════════════════════════════════════════════════" << std::endl;
  std::cout << "\nALTITUDE:" << std::endl;
  std::cout << " • Arrow Up / Down    : Increase/Decrease thrust" << std::endl;
  std::cout << " • h                  : Enable AUTO-HOVER (holds current altitude)" << std::endl;
  std::cout << " • SPACE              : Disable auto-hover (manual thrust)" << std::endl;
  std::cout << "\nATTITUDE (hold to tilt, release to AUTO-LEVEL):" << std::endl;
  std::cout << " • i / k              : Pitch forward/back" << std::endl;
  std::cout << " • j / l              : Roll left/right" << std::endl;
  std::cout << " • u / o              : Yaw left/right" << std::endl;
  std::cout << "\nSYSTEM:" << std::endl;
  std::cout << " • a                  : Arm/Disarm" << std::endl;
  std::cout << " • q                  : Quit" << std::endl;
  std::cout << "\n═══════════════════════════════════════════════════" << std::endl;
  std::cout << "\n FEATURES:" << std::endl;
  std::cout << " • IMU-based state estimation (200 Hz)" << std::endl;
  std::cout << " • Hardware-ready architecture" << std::endl;
  std::cout << " • AUTO-HOVER: Press 'h' to lock altitude" << std::endl;
  std::cout << " • Release keys → Auto-levels smoothly" << std::endl;
  std::cout << "═══════════════════════════════════════════════════\n" << std::endl;

  std::cout << "MeshCat URL: " << meshcat->web_url() << std::endl;
  std::cout << "\nPress Enter to start..." << std::endl;
  std::cin.get();

  drake::set_conio_terminal_mode();
  simulator.set_target_realtime_rate(1.0);

  // ========================================================================
  // SIMULATION LOOP VARIABLES
  // ========================================================================
  
  double sim_time = 0.0;
  const double dt = 0.005;
  const double end_time = 300.0;

  // User control state (NO MORE CONTROL LOGIC HERE!)
  bool auto_hover_enabled = true;
  double hover_target_altitude = 0.0;
  
  double target_roll = 0;
  double target_pitch = 0;
  double target_yaw = 0.0;  // deg/s
  
  const double max_angle = 0.9;
  const double angle_inc = 0.02;
  const double decay_factor = 0.90;
  
  int decay_counter = 0;
  
  bool armed = false;
  bool running = true;

  double last_print_time = 0.0;

  // Control input vector: NOW 5 ELEMENTS! [altitude, roll, pitch, yaw, mode]
  Eigen::VectorXd control_input(5);
  control_input.setZero();

  // ========================================================================
  // MAIN SIMULATION LOOP
  // ========================================================================
  
  while (running && sim_time < end_time) {
    int key = drake::get_key();

    // ARM / DISARM (unchanged)
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

    // QUIT 
    if (key == 'q' || key == 'Q') {
      std::cout << "\nQuit requested." << std::endl;
      break;
    }
    // ========================================================================
    // ALTITUDE TARGET CONTROL (Arrow keys)
    // ========================================================================
    if (key == drake::KEY_ARROW_UP) {
      hover_target_altitude += 0.1;
      std::cout << "Target altitude: " << hover_target_altitude << " m" << std::endl;
    } else if (key == drake::KEY_ARROW_DOWN) {
      hover_target_altitude -= 0.1;
      hover_target_altitude = std::max(hover_target_altitude, 0.0);
      std::cout << "Target altitude: " << hover_target_altitude << " m" << std::endl;
    }
    
    // ========================================================================
    // AUTO-HOVER TOGGLE
    // ========================================================================
    if (key == 'h' || key == 'H') {
      auto_hover_enabled = true;
      // Read current altitude from EKF output (NOT plant!)
      auto& ekf_context = diagram->GetSubsystemContext(*ekf, root_context);
      const auto& ekf_output = ekf->get_output_port(0).Eval(ekf_context);
      hover_target_altitude = ekf_output(12);  // Position Z from EKF
      std::cout << "\n*** AUTO-HOVER ENABLED at " << hover_target_altitude << "m ***" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
    
    if (key == ' ') {
      if (auto_hover_enabled) {
        auto_hover_enabled = false;
        std::cout << "\n*** AUTO-HOVER DISABLED ***" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
      }
    }

    // ========================================================================
    // ANGLE CONTROL (i/k/j/l/u/o keys)
    // ========================================================================
    
    
    // Apply decay ONLY if no keys are pressed
    bool pitch_key_pressed = (key == 'i' || key == 'I' || key == 'k' || key == 'K');
    bool roll_key_pressed = (key == 'j' || key == 'J' || key == 'l' || key == 'L');
    //bool yaw_key_pressed = (key == 'u' || key == 'U' || key == 'o' || key == 'O');
    
    if (!pitch_key_pressed && !roll_key_pressed && decay_counter > 30) {
      target_roll *= decay_factor;
      target_pitch *= decay_factor;
    }
    
    // User input in WORLD FRAME (what the pilot expects)
    double world_forward = 0.0;
    double world_right = 0.0;
    
    // Apply key inputs
    if (pitch_key_pressed){  
      decay_counter = 0; 
      if (key == 'i' || key == 'I') {
        world_forward += angle_inc;
      } else if (key == 'k' || key == 'K') {
        world_forward -= angle_inc;
      }
    }

    if (roll_key_pressed){  
      decay_counter = 0;
      if (key == 'j' || key == 'J') {
        world_right -= angle_inc;
      } else if (key == 'l' || key == 'L') {
        world_right += angle_inc;
      }
    }
  
    double current_yaw = 0.0;
    if (armed && sim_time > 0.01) {
      auto& current_plant_context = plant.GetMyMutableContextFromRoot(&root_context);
      const math::RigidTransformd current_pose = 
          plant.GetFreeBodyPose(current_plant_context, drone_body);
      const math::RollPitchYaw<double> current_rpy(current_pose.rotation());
      current_yaw = current_rpy.yaw_angle();
    }

    if (key == 'u' || key == 'U') {
      target_yaw += angle_inc ;
    } else if (key == 'o' || key == 'O') {
      target_yaw -= angle_inc ;
    }
  
    
    if (std::abs(target_roll) < 0.005) target_roll = 0.0;
    if (std::abs(target_pitch) < 0.005) target_pitch = 0.0;

    target_pitch += world_forward;
    target_roll +=  world_right;

    // Clamp to limits
    target_pitch = std::clamp(target_pitch, -max_angle, max_angle);
    target_roll = std::clamp(target_roll, -max_angle, max_angle);    

    // ========================================================================
    // BUILD CONTROL INPUT (5 ELEMENTS) - JUST COMMANDS
    // ========================================================================
    if (!armed) {
      control_input.setZero();
    } else {
      control_input(0) = hover_target_altitude;           // Target altitude
      control_input(1) = target_roll;                      // Target roll
      control_input(2) = target_pitch;                     // Target pitch
      control_input(3) = target_yaw;                       // Target yaw
      control_input(4) = auto_hover_enabled ? 1.0 : 0.0;   // ltitude mode
      if (roll_key_pressed){
        std::cout << "target roll:" << target_roll << std::endl;
      }
      if (pitch_key_pressed){
        std::cout << "target pitch:" << target_pitch << std::endl;
      }
    }


    // Send to controller
    root_context.FixInputPort(
        0,
        drake::Value<drake::systems::BasicVector<double>>(
            drake::systems::BasicVector<double>(control_input)));

    // // ========================================================================
    // // ADVANCE SIMULATION
    // // ========================================================================
    // if (armed) {
    //   simulator.AdvanceTo(sim_time + dt);
    //   sim_time += dt;
      
    //   // Print status every 1 second
    //   // Print status every 1 second
    //   if (sim_time - last_print_time >= 1.0) {
    //     // Read from EKF output (not IMU!) for status display
    //     auto& ekf_context = diagram->GetSubsystemContext(*ekf, root_context);
    //     const auto& ekf_output = ekf->get_output_port(0).Eval(ekf_context);
        
    //     // Parse EKF output: [quat(4), angular_vel(3), accel(3), pos(3)]
    //     Eigen::Quaterniond quat(ekf_output(0), ekf_output(1), 
    //                             ekf_output(2), ekf_output(3));
    //     const math::RotationMatrix<double> R_WB(quat);
    //     const math::RollPitchYaw<double> current_rpy(R_WB);
    //     const double current_altitude = ekf_output(12);  // Position Z
        
    //     std::cout << "t=" << std::fixed << std::setprecision(1) << sim_time;
        
    //     if (auto_hover_enabled) {
    //       std::cout << " [HOVER@" << std::setprecision(2) << hover_target_altitude << "m]";
    //     }
        
    //     std::cout << " | Alt=" << std::setprecision(2) << current_altitude << "m"
    //               << " | Angle=[" << std::setprecision(1)
    //               << current_rpy.roll_angle()*57.3 << "°, " 
    //               << current_rpy.pitch_angle()*57.3 << "°]" << std::endl;
        
    //     last_print_time = sim_time;
    //   }
    // } else {
    //   diagram->ForcedPublish(root_context);
    //   std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // }

    if (armed) {

      // NOW advance simulation AFTER debug print
      simulator.AdvanceTo(sim_time + dt);
      sim_time += dt;
      decay_counter += 1;
    } else {
      diagram->ForcedPublish(root_context);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  
  }  

  std::cout << "\nSimulation stopped." << std::endl;
  
  // Restore terminal
  struct termios term;
  tcgetattr(STDIN_FILENO, &term);
  term.c_lflag |= ICANON;
  term.c_lflag |= ECHO;
  tcsetattr(STDIN_FILENO, TCSANOW, &term);
  
  std::cout << "Press Enter to exit..." << std::endl;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  std::cin.get();
  return 0;
}