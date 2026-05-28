/**
 * @file Config.cpp
 * @brief Robot wiring and dimensions — start here when setting up a new robot.
 *
 * Quick setup (three steps):
 *   1. Set motor and sensor ports in @c Robot::Ports (match the numbers on the brain).
 *   2. Set measurements in @c Robot::Measurements (inches).
 *   3. Build and upload. If a motor spins backward, flip the sign on that port only.
 *
 * Tracking wheels you are not using: leave the port at @c 0.
 */

#include "Config.hpp"

// =============================================================================
// STEP 1 — Ports (numbers printed on the V5 brain next to each cable)
// =============================================================================

namespace Robot {
namespace Ports {

// --- Drivetrain: list every motor on each side (front → back) ---
inline constexpr int LEFT_FRONT = 4;
inline constexpr int LEFT_MIDDLE = -5;  // negative = wired / spins backward
inline constexpr int LEFT_BACK = -6;

inline constexpr int RIGHT_FRONT = 1;
inline constexpr int RIGHT_MIDDLE = -2;
inline constexpr int RIGHT_BACK = 3;

// --- Inertial sensor (IMU) ---
inline constexpr int IMU = 8;

// --- Tracking wheels (rotation sensors). Use 0 if that wheel is not on the robot. ---
inline constexpr int VERTICAL_WHEEL = -18;   // parallel to drive wheels (forward/back)
inline constexpr int HORIZONTAL_WHEEL = 0;   // sideways wheel — 0 = disabled

}  // namespace Ports

// =============================================================================
// STEP 2 — Physical sizes (all distances in inches)
// =============================================================================

namespace Measurements {

// Drivetrain
inline constexpr double TRACK_WIDTH = 11.5;        // left wheel center ↔ right wheel center
inline constexpr double DRIVE_WHEEL_DIAMETER = 3.25;

// Vertical tracking wheel (rolls when the robot drives forward/back)
inline constexpr double VERTICAL_WHEEL_DIAMETER = 2.0;
inline constexpr double VERTICAL_WHEEL_OFFSET =
    0.0;  // side-to-side distance from robot center (+ = toward right)

// Horizontal tracking wheel (rolls when the robot strafes / slides sideways)
inline constexpr double HORIZONTAL_WHEEL_DIAMETER = 2.75;
inline constexpr double HORIZONTAL_WHEEL_OFFSET =
    -4.75;  // front-to-back distance from robot center (+ = toward front)

}  // namespace Measurements

// =============================================================================
// STEP 3 — Built configs (usually leave this section alone)
// =============================================================================

const Vortex::DrivetrainConfig drivetrain{
    .left_ports = {Ports::LEFT_FRONT, Ports::LEFT_MIDDLE, Ports::LEFT_BACK},
    .right_ports = {Ports::RIGHT_FRONT, Ports::RIGHT_MIDDLE, Ports::RIGHT_BACK},
    .track_width_in = Measurements::TRACK_WIDTH,
    .wheel_diameter_in = Measurements::DRIVE_WHEEL_DIAMETER,
};

const Vortex::ImuConfig imu{.port = Ports::IMU};

const Vortex::TrackerConfig vertical_tracker{
    .port = Ports::VERTICAL_WHEEL,
    .diameter_in = Measurements::VERTICAL_WHEEL_DIAMETER,
    .offset_in = Measurements::VERTICAL_WHEEL_OFFSET,
};

const Vortex::TrackerConfig horizontal_tracker{
    .port = Ports::HORIZONTAL_WHEEL,
    .diameter_in = Measurements::HORIZONTAL_WHEEL_DIAMETER,
    .offset_in = Measurements::HORIZONTAL_WHEEL_OFFSET,
};

}  // namespace Robot

// =============================================================================
// Hardware objects (created from the tables above — do not edit)
// =============================================================================

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_drive(Robot::drivetrain.left_ports);
pros::MotorGroup right_drive(Robot::drivetrain.right_ports);

pros::Imu imu(Robot::imu.port);

Vortex::Wheel vertical_wheel(Robot::vertical_tracker);
Vortex::Wheel horizontal_wheel(Robot::horizontal_tracker);

Vortex::Odometry odom(Vortex::make_odom_sensors(
    &imu, &vertical_wheel,
    Robot::horizontal_tracker.enabled() ? &horizontal_wheel : nullptr));

Vortex::Chassis chassis(&left_drive, &right_drive, &odom, Robot::drivetrain);

Vortex::AutonSelector auton_selector;
Vortex::PIDTuner pid_tuner;
