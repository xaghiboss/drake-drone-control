# Quadcopter Simulation v3.0

**Version:** 3.0  
**Date:** January 25, 2026  
**Status:** ⚠️ Development (Yaw Issue Present)

## Overview

Version 3.0 adds X-configuration motor layout and auto-hover altitude hold. The drone can automatically maintain altitude using a PD controller.

## Key Features

- **X-Configuration** - Motors at 45° diagonals for enhanced maneuverability
- **Auto-Hover** - PD controller maintains altitude automatically
- **World-Frame Control** - Attempts intuitive directional control
- **Enhanced Stability** - Balanced decay factor (85% per frame)

## Motor Layout

Code
       Front
    Red   Blue
      \ X /
      / X \
  Yellow  Green
       Back
Code

## Controls

| Key | Function |
|-----|----------|
| `↑` / `↓` | Increase/Decrease thrust (manual) or adjust target altitude (auto-hover) |
| `h` | Enable auto-hover at current altitude |
| `SPACE` | Disable auto-hover (return to manual) |
| `i` / `k` | Pitch forward/backward |
| `j` / `l` | Roll left/right |
| `u` / `o` | Yaw left/right ⚠️ |
| `a` | Arm/Disarm motors |
| `q` | Quit |

## Auto-Hover System

**Enable:** Press `h` while flying  
**Adjust Altitude:** Use `↑`/`↓` (±10cm per press)  
**Disable:** Press `SPACE`

**PD Controller:**
- Kp = 3.0 (altitude error)
- Kd = 2.0 (vertical velocity damping)

## Movement Patterns (X-Config)

- **Forward:** Blue+Red ↑, Yellow+Green ↓
- **Backward:** Blue+Red ↓, Yellow+Green ↑
- **Left:** Blue+Yellow ↑, Red+Green ↓
- **Right:** Blue+Yellow ↓, Red+Green ↑

## PID Gains

**Outer Loop (Angle → Rate):**
- Roll/Pitch: 8.0
- Yaw: 1.5 (reduced for X-config stability)

**Inner Loop (Rate → Torque):**
- Roll/Pitch: P=0.5, D=0.1
- Yaw: P=0.15, D=0.05 (reduced)

## Technical Specs

- **Mass:** 0.5 kg
- **Hover Thrust:** 4.905 N
- **Arm Length:** 0.15 m (diagonal: 0.106 m)
- **Max Tilt:** 23° (0.4 rad)
- **Loop Rate:** 200 Hz (5ms timestep)
- **Initial Spawn:** 0° yaw

## Building and Running

```bash
bazel build //:quadcopter
./run.sh
Performance
✅ Stable hover with auto-hover enabled
✅ Altitude hold within ±5cm
✅ Smooth X-config flight
⚠️ Yaw limited to ±45° (crashes beyond)

Known Issues
⚠️ Critical: Yaw Instability Beyond ±45°
Problem: Drone crashes when yawing more than ±45° from initial heading.

Error:

Code
QuaternionToRotationMatrix(): All the elements in a quaternion are zero.
Workaround: Limit yaw inputs to ±30° for stable flight.

Status: Under investigation. Suspected causes:

World-to-body frame transformation misalignment
Yaw torque coupling into roll/pitch
Numerical instability in physics solver
What's New from v2.0
Added X-configuration motor layout
Added auto-hover altitude hold system
Added world-frame transformation (partial)
Increased max tilt from 20° to 23°
Reduced yaw gains for X-config stability
Changed spawn from 45° to 0° yaw
Differences from v2.0
Feature	v2.0	v3.0
Motor Config	+ (Plus)	X (Diagonal)
Auto-Hover	❌	✅
Yaw Range	360°	±45° ⚠️
Max Tilt	20°	23°
Spawn Yaw	0°	0°
Author: xaghiboss
Repository: https://github.com/xaghiboss/drake-drone-control
