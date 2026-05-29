---
layout: default
title: Introduction
---

# Vortex

Vortex is a C++ motion-control and odometry library for **VEX V5 competition robots**, built and maintained by Team **56S-Override**. It provides accurate pose estimation, a clean autonomous motion API, and on-robot tooling so teammates can configure hardware and write autons without touching the math.

---

## What's included

| System | Description |
|---|---|
| **Odometry** | Real-time `(x, y, θ)` pose tracking via tracking wheels and IMU fusion |
| **Chassis** | PID-based blocking motion: `drive_distance`, `turn_to_heading`, `drive_to_point`, `follow_path` |
| **Compat Layer** | LemLib & EZ-Template naming aliases — familiar API, same hardware |
| **Auton Selector** | Controller-driven routine picker with SD card persistence |
| **PID Tuner** | On-robot gain adjustment without recompiling |
| **Branch Manager** | `vx` CLI for committing, deploying, and rolling back across `dev` → `main` |

---

## Quick setup {#quick-setup}

### 1. Clone the repo

```bash
git clone https://github.com/jonahchang207/56S-Override.git
cd 56S-Override
```

### 2. Configure ports and measurements

All hardware configuration lives in one file — `src/Config.cpp`. Edit port constants and dimensions there; do not touch any other files.

```cpp
namespace Robot {
namespace Ports {
    inline constexpr int LEFT_FRONT      =  4;
    inline constexpr int LEFT_MIDDLE     = -5;   // negative = reversed
    inline constexpr int LEFT_BACK       = -6;
    inline constexpr int RIGHT_FRONT     =  1;
    inline constexpr int RIGHT_MIDDLE    = -2;
    inline constexpr int RIGHT_BACK      =  3;
    inline constexpr int IMU             =  8;
    inline constexpr int VERTICAL_WHEEL  = -18;
    inline constexpr int HORIZONTAL_WHEEL = 0;   // 0 = disabled
}
namespace Measurements {
    inline constexpr double TRACK_WIDTH            = 11.5;
    inline constexpr double DRIVE_WHEEL_DIAMETER   =  3.25;
    inline constexpr double VERTICAL_WHEEL_DIAMETER =  2.0;
    inline constexpr double VERTICAL_WHEEL_OFFSET  =  0.0;
}
}
```

Negate a port number to reverse its spin direction. Set `HORIZONTAL_WHEEL = 0` if no horizontal tracker is mounted.

### 3. Initialize in main.cpp

```cpp
void initialize() {
    odom.init(0.0, 0.0, 0.0);   // calibrates IMU, spawns background task
    chassis.init();
}
```

### 4. Build and upload

```bash
pros mu
```

---

## Writing an autonomous routine

```cpp
void autonomous() {
    odom.reset();
    chassis.drive_distance(24.0, 3000);      // 24 inches forward, 3 s timeout
    chassis.turn_to_heading(90.0, 2000);     // face right
    chassis.drive_to_point(24.0, 24.0, 4000);
}
```

All `snake_case` helpers block until the move settles or the timeout fires. For async motion see the [Compat Layer](odometry_explainer.html#lemlib--ez-template-compat) section.

---

## Coordinate system

```
          +Y  (field forward, 0°)
           ▲
           │
-X ◄───────┼───────► +X
 (270°)    │          (90°)
           ▼
          -Y  (field backward, 180°)
```

Origin is wherever you call `odom.reset()` (typically the robot's starting tile). Heading is clockwise degrees from `+Y`.

---

*Team 56S-Override — Jonah Chang*
