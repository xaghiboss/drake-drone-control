# Drake Quadcopter Simulation

A physics-based quadcopter flight simulator built with Drake robotics toolkit, featuring real-time 3D visualization and keyboard control.

---

## 📦 What's in This Project

### **Core Simulation Files**

- **`main.cc`** - Entry point; initializes MeshCat visualization, creates simulation loop, handles timing
- **`quadcopter_model.h/cc`** - Physics model implementing quadcopter dynamics (forces, torques, Newton-Euler equations)
- **`quadcopter_controller.h/cc`** - Control system that converts user input to motor commands
- **`keyboard_input.h/cc`** - Handles real-time keyboard input for manual flight control
- **`world.h/cc`** - Defines the simulation environment (ground plane, obstacles)

### **Build Configuration**

- **`MODULE.bazel`** - Declares dependencies (Drake, Eigen, rules_cc)
- **`BUILD.bazel`** - Defines compilation targets and links required Drake modules
- **`.bazelrc`** - Compiler flags (C++20 standard)
- **`build.sh`** - Build script with isolated Bazel cache
- **`run.sh`** - Convenience script to build and run

### **Documentation**

- **`CHANGELOG.md`** - Version history and feature evolution
- **`README.md`** - This file

---

## 🎯 Key Features

- **Physics Simulation**: 6-DOF rigid body dynamics with gravity, thrust, and aerodynamic effects
- **Motor Configurations**: Supports both + (aligned) and X (45° offset) quadcopter layouts
- **Real-time Control**: WASD keyboard controls with immediate response
- **3D Visualization**: Drake MeshCat web interface (localhost:7000)
- **Modular Design**: Separated model, controller, and input handling

---

## 🏗️ Architecture

User Input (Keyboard) ↓ Controller → Converts to motor speeds ↓ Physics Model → Calculates forces/torques ↓ State Integration → Updates position/orientation ↓ Visualization → Renders in MeshCat

Code

---

## 🚀 Quick Setup

### 1. Clone Repository
```bash
git clone https://github.com/xaghiboss/drake-drone-control.git
cd drake-drone-control/simulation/quadcopter_sim
```

### 2. Configure Drake Path
Edit build.sh:
```bash
nano build.sh
```
Update Drake path (around line 5):

```bash
OUT_ROOT="/home/YOUR_USERNAME/.bazel_output_quadcopter"
```
Edit MODULE.bazel:

```bash
nano MODULE.bazel
```
Update Drake installation path (around line 8):

```bash
local_path_override(
    module_name = "drake",
    path = "/YOUR/DRAKE/PATH",  # e.g., /home/username/drake
)
```

### 3. Build
```bash
./build.sh
```

### 4. Run
```bash
bazel run //:quadcopter
```

### 5. View Visualization
Open browser: http://localhost:7000

## 🎮 Controls
Key	Action	Key	Action
W	Forward	Space	Up
S	Backward	Shift	Down
A	Left	Q	Rotate Left
D	Right	E	Rotate Right
R	Reset	X	Emergency Stop
ESC	Exit

## 📐 Technical Details
Physics Implementation
Rigid body dynamics: Uses Newton-Euler equations
State vector: Position (x,y,z), velocity (vx,vy,vz), orientation (roll,pitch,yaw), angular velocity (ωx,ωy,ωz)
Time integration: Explicit Euler with dt=0.01s
Motor model: Thrust proportional to squared rotor speed
Dependencies
Drake: Robotics simulation framework (MeshCat, geometry, math utilities)
Eigen: Linear algebra (vectors, matrices, rotations)
C++20: Modern C++ features (auto, lambdas, smart pointers)
Motor Configurations
v1.0-v2.0: + configuration (motors aligned with body X/Y axes)
v3.0: X configuration (motors at ±45° for improved maneuverability)

## 📊 Version Summary
Version	Configuration	Key Features
v1.0	+ layout	Basic flight, manual control
v2.0	+ layout	Improved stability, PID tuning
v3.0	X layout	Better agility, experimental auto-hover
## 🛠️ Project Structure Explained
Code
```bash
quadcopter_sim/
├── main.cc                    # Simulation loop, MeshCat setup
├── quadcopter_model.{h,cc}    # Physics: F=ma, τ=Iα
├── quadcopter_controller.{h,cc} # Control logic: input → motors
├── keyboard_input.{h,cc}      # Terminal keyboard capture
├── world.{h,cc}               # Environment: ground, obstacles
├── BUILD.bazel                # Bazel build rules
├── MODULE.bazel               # External dependencies
├── build.sh                   # Build with isolated cache
└── run.sh                     # Build + run wrapper
```

## 🔍 Code Highlights
Quadcopter Model (quadcopter_model.cc)
Implements UpdatePhysics(dt): Integrates equations of motion
Motor mixing: Converts 4 motor speeds → net force + 3-axis torque
Collision detection: Stops simulation if z < 0 (ground contact)
Controller (quadcopter_controller.cc)
Maps keyboard input to desired thrust/torque
X-configuration motor mixer (45° rotation matrix)
Saturation limits prevent over-actuation
Visualization (main.cc)
Creates MeshCat server on port 7000
Adds quadcopter body (box) and rotors (cylinders)
Updates transforms at 100Hz for smooth animation

## ⚙️ Build System
Uses Bazel for reproducible builds:

Hermetic: All dependencies declared
Incremental: Only rebuilds changed files
Cross-platform: Works on Linux/macOS/Windows
Build output: bazel-bin/quadcopter (executable)

## 📚 Learning Outcomes
This project demonstrates:

Rigid body dynamics in 3D
Real-time control systems
Bazel build configuration
Drake robotics framework
C++ project organization (headers, source separation)
Visualization with MeshCat

## 🐛 Known Issues
v3.0: Yaw oscillation in auto-hover (X-configuration tuning needed)
General: Requires terminal focus for keyboard input

## 🔗 References
Drake Documentation
Bazel Build System
Quadcopter Dynamics 
Author: Zirgham, Basil
Repository: https://github.com/xaghiboss/drake-drone-control
