# 56S-Override

VEX V5 robot code for team **56S-Override**, built on
[PROS](https://pros.cs.purdue.edu/) and a small motion library called
**Vortex** that lives in `include/Vortex/` and `src/Vortex/`.

## Project layout

| Path                        | Description                                                                 |
| --------------------------- | --------------------------------------------------------------------------- |
| `src/main.cpp`              | Competition entry points (`initialize`, `autonomous`, `opcontrol`, ...).    |
| `src/Config.cpp`            | Definitions for every hardware port + Vortex object used by the robot.      |
| `include/Config.hpp`        | `extern` declarations matching `Config.cpp`.                                |
| `include/Vortex/odom.hpp`   | Pose + tracking-wheel odometry stack.                                       |
| `include/Vortex/chassis.hpp`| Differential-drive motion control (PID, paths, async motions, safety).      |
| `include/Vortex/auton_selector.hpp` | LCD / controller selector with optional live dashboard.            |
| `include/Vortex/pid_tuner.hpp`      | On-robot PID gain editor driven by the controller.                 |
| `Doxyfile`                  | Doxygen configuration for the API docs.                                     |

The matching `.cpp` files live under `src/Vortex/`.

## Building the firmware

This is a standard PROS project, so the usual workflow works:

```bash
pros make            # compile
pros mu              # build + upload to the brain
pros terminal        # open the serial terminal
```

You'll need PROS CLI 3.x and the V5 toolchain installed.

## Generating the API documentation

The public API of the Vortex library is documented with Doxygen comments
in the headers. To generate browsable HTML docs:

```bash
doxygen Doxyfile
open docs/html/index.html      # macOS
# or: xdg-open docs/html/index.html   # Linux
# or: start docs\html\index.html       # Windows
```

The `docs/` directory is git-ignored.

## Quick tour: robot config

Edit port tables once in `src/Config.cpp` under the `Robot::` namespace:

```cpp
namespace Robot {
const Vortex::DrivetrainConfig drivetrain{{4, -5, -6}, {1, -2, 3}, 11.5, 3.25};
const Vortex::ImuConfig imu{8};
const Vortex::TrackerConfig vertical_tracker{-18, 2.0, 0.0};
const Vortex::TrackerConfig horizontal_tracker{0, 2.75, -4.75};  // port 0 = off
}
```

Hardware objects (`left_drive`, `odom`, `chassis`, …) are built from those
tables automatically.

## Quick tour: startup

```cpp
void initialize() {
  odom.init();
  chassis.init();
  pid_tuner.bind_chassis(&chassis);
  pid_tuner.bind_defaults();
}
```

## Quick tour: autonomous movement

Blocking helpers — no empty param structs or `async = false`:

```cpp
void red_left_auton() {
  odom.reset();
  chassis.drive_distance(24.0, 3000);
  chassis.turn_to_heading(90.0, 2000);
  chassis.drive_to_point(24.0, 24.0, 4000);
  chassis.drive_to_pose(36.0, 24.0, 90.0, 3500);
  chassis.follow_path(Vortex::make_path({{0, 0, 100}, {0, 24, 110}}), 5000);
}
```

LemLib-style async motions (`moveToPoint`, `turnToHeading`, …) are still
available when you need them.

See `src/main.cpp` for a working example with two routines and the PID
tuner wired up.
