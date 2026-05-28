#pragma once

/**
 * @file Config.hpp
 * @brief Global robot hardware for 56S-Override.
 *
 * @par Where to configure the robot
 * Open @c src/Config.cpp and edit:
 * - @c Robot::Ports — motor and sensor port numbers from the V5 brain
 * - @c Robot::Measurements — track width, wheel diameters, tracker offsets
 *
 * Everything else in this header is created automatically from those values.
 */

#include "api.h"
#include "Vortex/auton_selector.hpp"
#include "Vortex/chassis.hpp"
#include "Vortex/odom.hpp"
#include "Vortex/pid_tuner.hpp"
#include "Vortex/robot_config.hpp"

namespace Robot {

/** Drive motors + chassis dimensions (see @c src/Config.cpp). */
extern const Vortex::DrivetrainConfig drivetrain;

/** IMU port (see @c src/Config.cpp). */
extern const Vortex::ImuConfig imu;

/** Forward/back tracking wheel; port @c 0 disables it. */
extern const Vortex::TrackerConfig vertical_tracker;

/** Left/right tracking wheel; port @c 0 disables it. */
extern const Vortex::TrackerConfig horizontal_tracker;

}  // namespace Robot

// --- Controllers ---

/** Primary V5 controller (driver + auton selector UI). */
extern pros::Controller master;

// --- Drivetrain motors ---

extern pros::MotorGroup left_drive;
extern pros::MotorGroup right_drive;

// --- Sensors ---

extern pros::Imu imu;

// --- Odometry & chassis ---

extern Vortex::Wheel vertical_wheel;
extern Vortex::Wheel horizontal_wheel;

extern Vortex::Odometry odom;
extern Vortex::Chassis chassis;

// --- Utilities ---

extern Vortex::AutonSelector auton_selector;
extern Vortex::PIDTuner pid_tuner;
