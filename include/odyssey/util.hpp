#pragma once

#include <vector>
#include "odyssey/pose.hpp"

namespace odyssey {

/**
 * @brief Which way the robot should rotate during a turn
 *
 * Directions are in the compass frame: CW = clockwise when viewed from above.
 */
enum class AngularDirection {
    CW_CLOCKWISE,
    CCW_COUNTERCLOCKWISE,
    AUTO // take whichever direction is shorter
};

/**
 * @brief Sign of a number: -1, 0, or 1
 */
template <typename T> constexpr int sgn(T value) { return (T(0) < value) - (value < T(0)); }

/** convert radians to degrees */
float radToDeg(float rad);

/** convert degrees to radians */
float degToRad(float deg);

/**
 * @brief Wrap an angle into [0, 360) degrees or [0, 2pi) radians
 *
 * @param angle the angle to wrap
 * @param radians true if the angle is in radians, false if degrees
 */
float sanitizeAngle(float angle, bool radians = true);

/**
 * @brief Shortest (or forced-direction) angular error between two angles
 *
 * The returned error has the same sign convention as the frame you pass in.
 * In the compass frame (degrees), a positive error means "turn clockwise".
 * In the math frame (radians), a positive error means "turn counterclockwise".
 *
 * @param target the target angle
 * @param position the current angle
 * @param radians true if the angles are in radians, false if degrees
 * @param direction AUTO for shortest path, or force CW/CCW (compass frame)
 */
float angleError(float target, float position, bool radians = true,
                 AngularDirection direction = AngularDirection::AUTO);

/** average of a vector of floats */
float avg(const std::vector<float>& values);

/**
 * @brief Exponential moving average
 *
 * @param current current value
 * @param previous previous output
 * @param smooth smoothing factor (0-1). 1 = no smoothing
 */
float ema(float current, float previous, float smooth);

/**
 * @brief Limit how fast a value is allowed to change (slew rate limiter)
 *
 * @param target the requested value
 * @param current the previous output
 * @param maxChange maximum change per call. 0 disables the limit
 */
float slew(float target, float current, float maxChange);

/**
 * @brief Signed curvature of the arc that starts at pose (tangent to its
 * heading) and passes through another point
 *
 * Positive curvature curves to the left (counterclockwise). pose.theta must
 * be in standard math radians.
 *
 * @return curvature in 1/inches. 0 if the points are coincident
 */
float getCurvature(Pose pose, Pose other);

} // namespace odyssey
