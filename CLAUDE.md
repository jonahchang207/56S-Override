# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## Build commands

The PROS CLI is installed at:
```
~/Library/Application\ Support/Code/User/globalStorage/sigbots.pros/install/pros-cli-macos/pros
```

The ARM toolchain (`arm-none-eabi-g++`) must be on PATH for builds to succeed; it is available when the user runs `pros m` from their normal terminal or via the VS Code PROS extension, but **not** in a plain shell session. If the toolchain is missing, suggest the user run the command from their terminal directly.

```bash
pros make          # compile only
pros mu            # compile and upload to the V5 brain
pros terminal      # stream printf output from the robot over USB
```

There are no unit tests — all validation is done on the physical robot.

---

## Repository layout

```
src/
  main.cpp          — PROS competition entry points (initialize / autonomous / opcontrol)
  Config.cpp        — ALL hardware ports and measurements; the only file teammates touch
  Vortex/
    odom.cpp        — pose-estimation math + background update task
    chassis.cpp     — PID motion execution
    auton_selector.cpp
    pid_tuner.cpp

include/
  Config.hpp        — extern declarations mirroring every object in Config.cpp
  Vortex/
    robot_config.hpp — plain config structs (DrivetrainConfig, TrackerConfig, etc.)
    odom.hpp         — Odometry + Wheel + OdomSensors public API
    chassis.hpp      — Chassis, PIDGains, MotionConfig, motion-param structs, Path API
    auton_selector.hpp
    pid_tuner.hpp
```

---

## Architecture

Everything in the project flows through two global objects defined in `Config.cpp` and declared `extern` in `Config.hpp`: `odom` (`Vortex::Odometry`) and `chassis` (`Vortex::Chassis`). All autonomous code calls methods on these globals; `main.cpp` only includes `main.h` (which includes `Config.hpp` transitively).

### Odometry (`Vortex::Odometry`)

Pose is `(x, y, theta)` where x/y are inches, theta is **radians stored internally**, clockwise from field +Y. Use `heading_deg()` / `set_heading_deg()` for human-readable angles.

Sensor inputs are bundled via `OdomSensors` (fluent builder) or `OdomConfig` (direct struct). The live sensors used by the project are:

- `vertical_wheel` — `Vortex::Wheel` backed by a `pros::Rotation` sensor on the port in `Ports::VERTICAL_WHEEL`
- `horizontal_wheel` — same, port `0` means disabled; `TrackerConfig::enabled()` gates construction
- `imu` — `pros::Imu` on `Ports::IMU`

`make_odom_sensors()` in `Config.cpp` wires these into an `OdomSensors` bundle and skips any tracker whose port is `0`.

`Odometry::init()` must be called in `initialize()` — it resets the IMU, zeros encoders, sets pose to `(0, 0, 0)`, and spawns the 10 ms background update task.

### Chassis (`Vortex::Chassis`)

Owns two `pros::MotorGroup*` pointers (left/right), an `Odometry*`, a `MotionConfig` (nine `PIDGains` structs + motion-profile parameters), and a `SafetyConfig`.

**Motion API has two styles:**

| Style | Example | Blocks? |
|---|---|---|
| Snake_case helpers | `chassis.drive_distance(24.0, 3000)` | Always blocking |
| CamelCase LemLib/EZ compat | `chassis.moveToPoint(x, y, t, params, async)` | Async by default; call `waitUntilDone()` |

All autonomous in `main.cpp` currently uses the blocking snake_case API.

Path following uses pure-pursuit via `chassis.follow_path(make_path({...}), timeout, params)`. `inject_path` and `smooth_path` are applied inside `FollowParams` automatically when `smooth = true`.

### Port type constraint

`pros::MotorGroup` requires `std::vector<std::int8_t>`, but `DrivetrainConfig` stores `std::vector<int>` for ergonomics. The helper `Vortex::to_motor_ports(const std::vector<int>&)` in `robot_config.hpp` performs the conversion and must be used wherever a `DrivetrainConfig` port list is passed to a `MotorGroup` constructor.

### AutonSelector

Routines are registered in `initialize()` with `.add("Name", fn)`. The selector persists the chosen routine to `/usd/auton.txt` on the SD card via `save_selected()` / `load_selected()`. `start_dashboard()` spawns a background task that renders pose, target, battery, and motor temperatures on the V5 LCD.

### PIDTuner

`pid_tuner.bind_defaults()` registers all nine `MotionConfig` PID loops automatically. The tuner is controller-driven — no code changes needed to adjust gains between runs. Call `pid_tuner.iterate(master)` inside `opcontrol()` if live tuning is desired (currently not called in `main.cpp`).

---

## Robot configuration workflow

1. Edit port constants in `Robot::Ports` (top of `src/Config.cpp`).
2. Edit dimensions in `Robot::Measurements` if wheels or track width changed.
3. Negate a port (e.g. `4` → `-4`) if that motor spins backward; do not change anything else.
4. Set `HORIZONTAL_WHEEL = 0` if no horizontal tracking wheel is installed.
5. `pros mu` to upload.
