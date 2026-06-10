#pragma once

#include "odyssey/pose.hpp"
#include "odyssey/trackingwheel.hpp"
#include "pros/imu.hpp"

namespace odyssey {

/**
 * @brief The sensors used for odometry
 *
 * Any pointer may be nullptr if the sensor doesn't exist. Recommended
 * setups, in order of accuracy:
 *  1. imu + 1-2 vertical tracking wheels + 1 horizontal tracking wheel
 *  2. imu + 1 vertical tracking wheel
 *  3. imu only (drive motor encoders are substituted automatically)
 *  4. two parallel vertical tracking wheels, no imu (heading from wheels)
 */
struct OdomSensors {
        /** vertical (forward-facing) tracking wheel */
        TrackingWheel* vertical1 = nullptr;
        /** second vertical tracking wheel */
        TrackingWheel* vertical2 = nullptr;
        /** horizontal (sideways-facing) tracking wheel */
        TrackingWheel* horizontal1 = nullptr;
        /** second horizontal tracking wheel */
        TrackingWheel* horizontal2 = nullptr;
        /** V5 inertial sensor */
        pros::Imu* imu = nullptr;
};

/**
 * @brief Set the sensors odometry will use. Called by Chassis::calibrate
 */
void setSensors(OdomSensors sensors);

/**
 * @brief The robot's current pose
 *
 * @param radians false (default): theta in compass degrees (0 = +y,
 *        clockwise positive). true: theta in standard math radians
 *        (0 = +x, counterclockwise positive)
 */
Pose getPose(bool radians = false);

/**
 * @brief Override the robot's pose (e.g. at the start of autonomous)
 *
 * @param pose the new pose
 * @param radians same angle convention as getPose
 */
void setPose(Pose pose, bool radians = false);

/**
 * @brief Run a single odometry update step. Called automatically by the
 * background tracking task - you should not need to call this yourself
 */
void update();

/**
 * @brief Reset sensor baselines and start the background tracking task.
 * Called by Chassis::calibrate
 */
void initOdometry();

} // namespace odyssey
