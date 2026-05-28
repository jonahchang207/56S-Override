#pragma once

/**
 * @file robot_config.hpp
 * @brief Port numbers and physical dimensions for your robot.
 *
 * Edit @c src/Config.cpp — that is the only file most teammates need to touch.
 * Values here are plain structs so you can use named fields (e.g. @c .port = 8)
 * instead of remembering argument order.
 */

#include <cstdint>
#include <vector>

namespace Vortex {

/**
 * @struct DrivetrainConfig
 * @brief Drive motor ports and wheel geometry.
 *
 * @par Motor ports
 * Use the number shown on the V5 brain next to each motor port.
 * If a side drives backward when it should go forward, negate that port
 * (@c 4 → @c -4).
 *
 * @par Example (in Config.cpp)
 * @code
 * const DrivetrainConfig drivetrain{
 *     .left_ports = {4, -5, -6},
 *     .right_ports = {1, -2, 3},
 *     .track_width_in = 11.5,
 *     .wheel_diameter_in = 3.25,
 * };
 * @endcode
 */
struct DrivetrainConfig {
  std::vector<int> left_ports{};
  std::vector<int> right_ports{};
  double track_width_in = 11.5;      ///< Center-to-center, left wheels ↔ right wheels (in).
  double wheel_diameter_in = 3.25;  ///< Driven wheel diameter (in).
  double gear_ratio = 1.0;          ///< Motor rotations per wheel rotation (e.g. 36:60 → 0.6).
};

/**
 * @struct ImuConfig
 * @brief Inertial sensor (IMU) port.
 */
struct ImuConfig {
  int port = 0;  ///< Brain port; must match the cable label on the robot.
};

/**
 * @struct TrackerConfig
 * @brief One odometry tracking wheel (rotation sensor + geometry).
 *
 * Set @c port to @c 0 to disable that wheel. Otherwise use the brain port
 * number (negate if the sensor counts the wrong direction).
 *
 * @par Offsets
 * Measure from the robot’s center to the wheel contact point:
 * - Vertical (drive-parallel) wheel: positive = toward the front of the robot.
 * - Horizontal (sideways) wheel: positive = toward the right side.
 */
struct TrackerConfig {
  int port = 0;                 ///< Rotation sensor port; @c 0 = not installed.
  double diameter_in = 2.0;   ///< Wheel diameter (in).
  double offset_in = 0.0;     ///< Distance from robot center to wheel (in).
  double gear_ratio = 1.0;    ///< Sensor rotations per wheel rotation.
  bool reversed = false;      ///< Flip reported spin direction without negating @c port.

  /** @brief @c true when this tracker is wired and should be used for odometry. */
  bool enabled() const { return port != 0; }
};

}  // namespace Vortex
