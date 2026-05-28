# Vortex Odometry System: Complete Technical & API Guide
## Team 56S-Override — AP Computer Science A Final Project
**Developer:** Jonah Chang  

---

## 1. Executive Summary & Design Philosophy
Odometry (often shortened to "odom") is the process of using sensors to estimate the change in position of a robot over time. In a VEX V5 robotics competition context, a reliable odometry system provides the robot's real-time coordinate position—denoted by a **Pose** $(X, Y, \theta)$—on the $12 \times 12$ foot field.

The **Vortex** odometry stack was designed around three core principles:
1. **High Abstraction**: Teammates can write autonomous movements using simple commands (e.g. `chassis.drive_to_point(24.0, 24.0)`) without understanding the complex trigonometric equations running in the background.
2. **Robustness Against Physical Glitches**: Embedded hardware is messy. The V5 Smart Ports communicate asynchronously, introducing latencies that break standard schoolbook algorithms. Vortex implements custom software filters to eliminate these physical bugs.
3. **User Friendliness**: Configuring a new robot is consolidated into a single configuration table in `src/Config.cpp`, and operating the system during runs is simple and clean.

---

## 2. Coordinate System & Field Conventions
To work with the Vortex API, it is essential to understand the field coordinate system:

* **Origin $(0,0)$**: Can be set anywhere on the field, but is typically the robot's starting location.
* **X-Axis**: Represents the side-to-side coordinate of the field.
  * $+X$ is to the **driver's right**.
  * $-X$ is to the **driver's left**.
* **Y-Axis**: Represents the forward-backward coordinate of the field.
  * $+Y$ is **straight forward** (away from the driver station).
  * $-Y$ is **straight backward** (toward the driver station).
* **Heading ($\theta$)**: Clockwise degrees or radians starting from $+Y$ (0 degrees/radians faces straight forward).
  * **90°** = facing driver's right ($+X$).
  * **180°** = facing driver's station ($-Y$).
  * **270° / -90°** = facing driver's left ($-X$).

```
                +Y (Field Forward / 0 deg)
                            ▲
                            │
      -X (Left / 270 deg) ◄─┼─► +X (Right / 90 deg)
                            │
                            ▼
                -Y (Field Backward / 180 deg)
```

---

## 3. Mathematical Mechanics of Pose Estimation
Vortex tracks the robot's movement using a **First-Order Circular Arc Approximation**. 

Every 10 milliseconds, a background thread runs the `update()` loop. It measures the linear travel distance on the robot's tracking wheels and the heading change, computes the movement relative to the robot's local frame, and transforms it into the global field frame.

### Step 1: Compute Sensor Deltas
The software queries the linear inches traveled by each active tracking wheel since the last loop iteration:
$$\Delta d_{\text{vertical}} = d_{\text{vertical}} - d_{\text{vertical, last}}$$
$$\Delta d_{\text{horizontal}} = d_{\text{horizontal}} - d_{\text{horizontal, last}}$$

### Step 2: Compute Heading Change ($\Delta\theta$)
If an Inertial Measurement Unit (IMU) is configured and enabled, the change in heading is computed directly from the gyroscope:
$$\Delta\theta = \theta_{\text{imu}} - \theta_{\text{imu, last}}$$

If no IMU is present, Vortex falls back on a differential vertical wheel pair (or the left and right drive wheels):
$$\Delta\theta = \frac{\Delta d_{\text{vertical, 1}} - \Delta d_{\text{vertical, 2}}}{\text{offset}_{\text{vertical, 2}} - \text{offset}_{\text{vertical, 1}}}$$

### Step 3: Compute Local Displacements
Because the robot may be turning while it moves, it travels in a circular arc rather than a straight line. The local displacements ($\Delta x_{\text{local}}$ and $\Delta y_{\text{local}}$) account for the rotation of the robot's sensors:
* **Forward Movement**: The average of the vertical tracking wheels corrected for rotation:
  $$\Delta y_{\text{local}} = \frac{\Delta d_{\text{vertical}} + \text{offset}_{\text{vertical}} \cdot \Delta\theta}{n_{\text{vertical}}}$$
* **Sideways Movement (Strafe)**: The average of the horizontal tracking wheels corrected for rotation:
  $$\Delta x_{\text{local}} = \frac{\Delta d_{\text{horizontal}} - \text{offset}_{\text{horizontal}} \cdot \Delta\theta}{n_{\text{horizontal}}}$$

### Step 4: Transform to Global Coordinates
Using the average heading during the step ($\theta_{\text{mid}} = \theta_{\text{last}} + \frac{\Delta\theta}{2}$), the local coordinate changes are rotated into the global field coordinates:
$$\Delta X = \Delta x_{\text{local}} \cdot \cos(\theta_{\text{mid}}) + \Delta y_{\text{local}} \cdot \sin(\theta_{\text{mid}})$$
$$\Delta Y = \Delta y_{\text{local}} \cdot \cos(\theta_{\text{mid}}) - \Delta x_{\text{local}} \cdot \sin(\theta_{\text{mid}})$$

These deltas are then added to the robot's accumulated pose:
$$X \leftarrow X + \Delta X$$
$$Y \leftarrow Y + \Delta Y$$
$$\theta \leftarrow \theta + \Delta\theta$$

---

## 4. The 3 Embedded Bugs We Solved
Standard robotics tutorials suggest simple algorithms that assume sensors respond instantly and perfectly. In the real world, VEX V5 hardware introduces asynchronous bugs. Vortex implements custom software filters to solve three of the most famous VEX sensor bugs:

### Bug 1: The Asynchronous Tare Spike Bug
> [!CAUTION]
> **The Problem:** Calling a hardware tare (e.g. `sensor->reset_position()`) is an asynchronous command sent over the V5 Smart Port. When a user starts their autonomous routine, the software calls tare and then immediately captures the "baseline" sensor positions. Because of communication lag, the sensor still reports the old non-zero value, which is saved as the baseline. A few milliseconds later, the sensor resets to zero, causing a massive, artificial delta spike. The robot thinks it has jumped several feet instantly, causing the autonomous code to spin out of control.

**The Vortex Solution:** We eliminated hardware taring completely! `RotationDistanceSensor` and `MotorGroupDistanceSensor` now implement a **Software-Based Tare Offset**.
* When `reset()` is called, we capture the current raw sensor reading in memory as `offset_degrees_`.
* In `get_distance_in()`, we subtract `offset_degrees_` from the live raw sensor reading.
This is 100% synchronous, instant, and completely immune to Smart Port lag.

### Bug 2: The Stale IMU Overwrite Bug
> [!WARNING]
> **The Problem:** Similar to taring encoders, setting the absolute rotation of the IMU (e.g. `imu->set_rotation(90.0)`) is an asynchronous command. If a user sets the robot's pose to a starting coordinate, and the code immediately reads `imu->get_rotation()` to establish the tracking baseline, it receives the old, stale heading value, instantly overwriting the starting pose.

**The Vortex Solution:** The IMU's internal absolute rotation is now treated as read-only. Instead of changing the IMU's internal state, Vortex stores a relative baseline `last_imu_rotation_rad_` and only measures the change in orientation ($\Delta\theta$) at each tick.
* When `set_pose()` is called, the target heading is written directly to `pose_.theta`, and `last_imu_rotation_rad_` is synchronously set to the current reading of the IMU.
* There is no asynchronous waiting, no telemetry lag, and starting coordinates are guaranteed to set instantly and accurately.

### Bug 3: The IMU Calibration Settle Race Condition
> [!IMPORTANT]
> **The Problem:** When the IMU is reset to calibrate, it takes up to 100–200 milliseconds to initialize and report that it is calibrating. If the code immediately checks `imu->is_calibrating()`, it receives `false` before the IMU has officially entered calibration. The software skips the waiting loop, starting the autonomous routine while the IMU is still calibrating, resulting in frozen or corrupt heading data.

**The Vortex Solution:** We added a robust 200ms delay immediately after calling `imu->reset()` inside `Odometry::calibrate()`:
```cpp
config_.imu->reset();
pros::delay(200); // Wait for the IMU to initiate calibration
while (config_.imu->is_calibrating()) {
  pros::delay(10);
}
```
This guarantees the sensor has transitioned into its calibration routine before the program is allowed to continue.

---

## 5. Software Architecture & OOP Principles (AP CSA Connections)
The Vortex architecture is an excellent showcase of Object-Oriented Programming (OOP) concepts tested in the AP CSA curriculum:

```
                  ┌────────────────────────┐
                  │ <<interface>>          │
                  │ DistanceSensor         │
                  └────────────────────────┘
                               ▲
                      ┌────────┴────────┐
                      │                 │
        ┌────────────────────────┐   ┌────────────────────────┐
        │ RotationDistanceSensor │   │ MotorGroupDistanceSensor│
        └────────────────────────┘   └────────────────────────┘
                      ▲                 ▲
                      └────────┬────────┘
                               │ (wrapped in)
                  ┌────────────────────────┐
                  │ Wheel                  │
                  └────────────────────────┘
```

### Abstraction & Polymorphism: The `DistanceSensor` Interface
The odometry equation doesn't care whether the robot tracks distance using three-wire encoders, smart rotation sensors, or drive motors. It only needs to know how many linear inches have been traveled.
* **Interface Abstraction**: We defined a pure virtual class `DistanceSensor` in `odom.hpp`.
* **Polymorphism**: `RotationDistanceSensor` and `MotorGroupDistanceSensor` inherit from this base interface. The `Odometry` class interacts solely with the base `DistanceSensor` pointers, demonstrating polymorphism—any subclass object can be used wherever the superclass interface is expected.

---

## 6. Physical Measurement Guide
For accurate tracking, the physical offsets of the wheels relative to the robot's center must be measured carefully.

```
                      Front (+Y)
                          │
         ┌────────────────┴────────────────┐
         │           Vertical Wheel        │
         │           Offset (Side-to-Side) │
         │           ◄────────►            │
         │          ┌─────────┐            │
         │          │  [===]  │            │
         │          └─────────┘            │
         │                                 │
  Left   ├─────── Robot Center (0,0) ──────┤   Right
  (-X)   │                                 │   (+X)
         │                                 │
         │    ┌─────────┐                  │
         │    │  [===]  │ Horizontal Wheel │
         │    └─────────┘ Offset           │
         │    ▲           (Front-to-Back)  │
         │    │                            │
         └────┼────────────────────────────┘
              │
          Back (-Y)
```

### 1. Vertical Tracking Wheel Offset (`VERTICAL_WHEEL_OFFSET`)
* **What it is:** The **side-to-side** distance from the center of the robot to the tracking wheel.
* **How to measure:** Use a ruler/tape measure to measure from the exact center of the drivetrain to the wheel.
* **Sign Convention:** 
  * If the wheel is to the **right** of the center, the value is **positive** ($+$).
  * If the wheel is to the **left** of the center, the value is **negative** ($-$).

### 2. Horizontal Tracking Wheel Offset (`HORIZONTAL_WHEEL_OFFSET`)
* **What it is:** The **front-to-back** distance from the center of the robot to the horizontal wheel.
* **How to measure:** Measure from the exact center of the drivetrain to the wheel along the forward axis.
* **Sign Convention:**
  * If the wheel is in **front** of the center, the value is **positive** ($+$).
  * If the wheel is **behind** the center, the value is **negative** ($-$).

---

## 7. API Reference & Code Examples
Operating the Vortex Odometry is simple, clean, and highly intuitive.

### Step 1: Wiring & Hardware Configuration
To customize your robot, open `src/Config.cpp` and set the physical values:

```cpp
namespace Robot {
namespace Ports {
    inline constexpr int LEFT_FRONT = 4;
    inline constexpr int LEFT_MIDDLE = -5;
    inline constexpr int LEFT_BACK = -6;
    
    inline constexpr int RIGHT_FRONT = 1;
    inline constexpr int RIGHT_MIDDLE = -2;
    inline constexpr int RIGHT_BACK = 3;
    
    inline constexpr int IMU = 8;
    inline constexpr int VERTICAL_WHEEL = -18;  // Negative reverses rotation direction
    inline constexpr int HORIZONTAL_WHEEL = 0;   // 0 = disabled
}

namespace Measurements {
    inline constexpr double TRACK_WIDTH = 11.5;        // Inches
    inline constexpr double DRIVE_WHEEL_DIAMETER = 3.25; // Inches
    inline constexpr double VERTICAL_WHEEL_DIAMETER = 2.0;
    inline constexpr double VERTICAL_WHEEL_OFFSET = 0.0; // Right/left offset
}
}
```

### Step 2: Operating the API in `main.cpp`
Inside `src/main.cpp`, control the robot's lifecycle and print coordinate logs:

```cpp
#include "main.h"

void initialize() {
  printf("Initializing Vortex...\n");
  
  // 1. One-call startup: calibrates IMU, captures sensor baselines, 
  //    and spawns the background thread loop!
  odom.init(0.0, 0.0, 0.0); // Starts at (0,0) facing 0 degrees (forward)
  chassis.init();
}

void autonomous() {
  // 2. Easily reset the robot's coordinate systems before an auton run
  odom.reset(12.0, 12.0, 90.0); // Reset to x=12", y=12", heading=90° (facing right)

  // 3. Command motion relative to the odometry coordinate frame
  chassis.drive_distance(24.0, 3000);       // Move 24 inches forward
  chassis.turn_to_heading(180.0, 2000);     // Turn to face backward
  chassis.drive_to_point(0.0, 0.0, 4000);   // Return back to starting position!
}

void opcontrol() {
  while (true) {
    // 4. Standard arcade drive using controller sticks
    const int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    chassis.arcade(throttle, turn);

    // 5. Extremely friendly printing helper!
    //    Prints x, y, and heading beautifully in one line to the terminal.
    odom.print(); 

    pros::delay(10);
  }
}
```

---

## 8. Summary of API Methods

| Method / Struct | Description | Usage Example |
| :--- | :--- | :--- |
| `odom.init(x, y, heading)` | Calibrates sensors, sets the initial pose, and spawns the background fusion task. | `odom.init(0, 0, 0);` |
| `odom.reset(x, y, heading)` | Resets the coordinate tracker to a specific pose. | `odom.reset(24.0, 48.0, 90.0);` |
| `odom.get_pose()` | Returns a `Pose` struct containing the current $X$, $Y$, and $\theta$ (in radians). | `Vortex::Pose pos = odom.get_pose();` |
| `odom.x()` | Returns the current $X$ coordinate in inches. | `double current_x = odom.x();` |
| `odom.y()` | Returns the current $Y$ coordinate in inches. | `double current_y = odom.y();` |
| `odom.heading_deg()` | Returns the current heading in degrees ($0 \dots 360$ CW from $+Y$). | `double degrees = odom.heading_deg();` |
| `odom.print()` | Prints formatted pose telemetry beautifully in the PROS console. | `odom.print();` |
| `odom.set_pose(x, y, heading)` | Changes coordinates instantly without restarting the background thread. | `odom.set_pose(0, 0, 90);` |
