# Quadcopter Simulation v2.0

**Version:** 2.0  
**Date:** January 24, 2026  
**Status:** ✅ Stable Release

## Overview

Version 2.0 introduces cascaded PID control with auto-stabilization. The drone automatically levels when you release the controls, making it easy to fly like a DJI drone.

## Key Features

- **Cascaded PID Control** - Two-loop architecture (angle → rate → torque)
- **Auto-Leveling** - Release keys and drone returns to level flight
- **Full 3-Axis Control** - Pitch, roll, and yaw
- **Plus (+) Configuration** - Motors on cardinal axes
- **Stable Hover** - No oscillations or drift

## Motor Layout

Code
     Front (0)
          |
Left (3) ----+---- Right (1) | Back (2)

Code

## Controls

| Key | Function |
|-----|----------|
| `↑` / `↓` | Increase/Decrease thrust |
| `i` / `k` | Pitch forward/backward |
| `j` / `l` | Roll left/right |
| `u` / `o` | Yaw left/right |
| `a` | Arm/Disarm motors |
| `q` | Quit |

## Flight Behavior

**Auto-Stabilization:**
- Hold key → Drone tilts → Moves
- Release key → Drone auto-levels → Stops

**Decay Rate:** 80% per frame (20% decay)  
**Auto-Level Time:** ~0.3 seconds

## PID Gains

**Outer Loop (Angle → Rate):**
- Roll/Pitch: 8.0
- Yaw: 3.0

**Inner Loop (Rate → Torque):**
- Roll/Pitch: P=0.5, D=0.1
- Yaw: P=0.3, D=0.08

## Technical Specs

- **Mass:** 0.5 kg
- **Hover Thrust:** 4.905 N
- **Arm Length:** 0.15 m
- **Max Tilt:** 20° (0.35 rad)
- **Loop Rate:** 200 Hz (5ms timestep)

## Building and Running

```bash
bazel build //:quadcopter
./run.sh
Performance
✅ Stable hover (±0.05m drift)
✅ Smooth auto-leveling (0.3s)
✅ Full 360° yaw rotation
✅ No oscillations

What's New from v1.0

Added cascaded PID control
Added auto-stabilization
Added pitch/roll/yaw control
Improved from 10ms to 5ms timestep
Changed from rate control to angle control

Known Limitations

No altitude hold (manual thrust only)
No position hold
No GPS-like features

Author: xaghiboss
Repository: https://github.com/xaghiboss/drake-drone-control 
