# 56S-Override

Competition robot code for VEX V5 team **56S-Override**, written in C++ on the [PROS](https://pros.cs.purdue.edu/) framework. The robot uses a custom motion library called **Vortex** that handles where the robot is on the field and how it drives itself during autonomous.

**[View the full documentation →](https://jonahchang207.github.io/56S-Override/)**

---

## What Vortex does

Vortex gives the robot two main capabilities:

- **Odometry** — tracks the robot's position (x, y, heading) in real time using a rotation sensor and an inertial sensor. The robot always knows where it is on the field.
- **Motion control** — lets you write autonomous routines using simple commands like "drive forward 24 inches" or "go to this coordinate." All the PID math runs in the background.

---

## Setting up the robot

Open `src/Config.cpp` and fill in the port numbers from your V5 brain. That's the only file you need to edit for a new robot.

```cpp
// Motor ports — use negative numbers if a motor spins the wrong way
inline constexpr int LEFT_FRONT  =  4;
inline constexpr int LEFT_MIDDLE = -5;
inline constexpr int LEFT_BACK   = -6;

inline constexpr int RIGHT_FRONT  = 1;
inline constexpr int RIGHT_MIDDLE = -2;
inline constexpr int RIGHT_BACK   =  3;

// Sensors
inline constexpr int IMU             =  8;
inline constexpr int VERTICAL_WHEEL  = -18;  // rotation sensor, negative = reversed
inline constexpr int HORIZONTAL_WHEEL =  0;  // 0 = not installed
```

Everything else — motor groups, odometry, chassis — is built from those values automatically.

---

## Building and uploading

Standard PROS workflow:

```bash
pros make          # compile
pros mu            # compile and upload to the brain
pros terminal      # read console output from the robot
```

You need PROS CLI 3.x and the VEX V5 toolchain installed. If you use VS Code, the PROS extension handles this with buttons.

---

## Writing autonomous routines

Start each routine by resetting the odometry, then call motion commands. They block until the move finishes (or times out), so you write them top to bottom like a script.

```cpp
void my_auton() {
  odom.reset();
  chassis.drive_distance(24.0, 3000);      // drive 24 inches, 3 second timeout
  chassis.turn_to_heading(90.0, 2000);     // turn to face 90 degrees
  chassis.drive_to_point(24.0, 24.0, 4000); // go to field coordinate (24, 24)
}
```

Register your routine in `initialize()` so it shows up in the auton selector:

```cpp
void initialize() {
  odom.init();
  chassis.init();
  auton_selector.add("My Auton", my_auton);
  auton_selector.bind_dashboard(&odom, &chassis, &master);
  auton_selector.start_dashboard();
}
```

Use the controller D-pad to pick which routine runs before the match.

---

## Project structure

```
src/
  main.cpp          — competition entry points (initialize, autonomous, opcontrol)
  Config.cpp        — all hardware ports and measurements in one place
  Vortex/
    odom.cpp        — odometry math and background update task
    chassis.cpp     — PID motion control
    auton_selector.cpp
    pid_tuner.cpp

include/
  Config.hpp        — extern declarations for everything in Config.cpp
  Vortex/
    odom.hpp        — public API for pose tracking
    chassis.hpp     — public API for motion commands
    robot_config.hpp — config struct definitions
```

---

*Team 56S-Override · Jonah Chang · Built with PROS*
