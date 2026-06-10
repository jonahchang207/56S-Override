#pragma once

#include "pros/rotation.hpp"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"

namespace odyssey {

/**
 * @brief Real-world diameters of common VEX omni wheels, in inches
 *
 * Wheels are rarely the size printed on the box. Using these measured
 * values instead of the nominal size noticeably improves tracking accuracy.
 */
namespace Omniwheel {
constexpr float NEW_2 = 2.125;
constexpr float NEW_275 = 2.75;
constexpr float OLD_275 = 2.75;
constexpr float NEW_325 = 3.25;
constexpr float NEW_4 = 4.0;
constexpr float OLD_4 = 4.18;
} // namespace Omniwheel

/**
 * @brief A tracking wheel: a wheel + encoder used to measure distance traveled
 *
 * Can be a dedicated tracking wheel with a V5 Rotation sensor or ADI (red)
 * optical shaft encoder, or the drivetrain motors' built-in encoders.
 *
 * Offset sign conventions (distance from the tracking center, in inches):
 *  - vertical wheels (measure forward travel): negative = left of center,
 *    positive = right of center
 *  - horizontal wheels (measure sideways travel): negative = behind center,
 *    positive = in front of center
 */
class TrackingWheel {
    public:
        /**
         * @brief Tracking wheel measured by a V5 Rotation sensor
         *
         * @param encoder pointer to the rotation sensor. Reverse the sensor
         *        in its own constructor if it counts the wrong way
         * @param wheelDiameter diameter of the wheel, inches. See the
         *        odyssey::Omniwheel constants
         * @param offset signed distance from the tracking center, inches
         * @param gearRatio wheel revolutions per sensor revolution.
         *        1 if the sensor is on the wheel axle (default)
         */
        TrackingWheel(pros::Rotation* encoder, float wheelDiameter, float offset,
                      float gearRatio = 1);

        /**
         * @brief Tracking wheel measured by an ADI optical shaft encoder
         */
        TrackingWheel(pros::adi::Encoder* encoder, float wheelDiameter, float offset,
                      float gearRatio = 1);

        /**
         * @brief "Tracking wheel" backed by drive motor encoders
         *
         * Less accurate than a dedicated tracking wheel because drive wheels
         * slip, but requires no extra hardware. Used automatically as a
         * fallback by Chassis::calibrate when no vertical tracking wheels
         * are configured.
         *
         * @param motors pointer to the drive motor group on one side
         * @param wheelDiameter diameter of the drive wheels, inches
         * @param offset signed distance from the tracking center, inches
         *        (usually +/- trackWidth / 2)
         * @param driveRpm output rpm of the drive wheels (after gearing)
         */
        TrackingWheel(pros::MotorGroup* motors, float wheelDiameter, float offset,
                      float driveRpm);

        /**
         * @brief Reset the encoder to 0
         */
        void reset();

        /**
         * @brief Total distance traveled by the wheel since the last reset,
         * in inches
         */
        float getDistanceTraveled();

        /**
         * @brief Signed offset from the tracking center, inches
         */
        float getOffset() const;

    protected:
        enum class Type { ROTATION, ADI_ENCODER, MOTOR };

        Type type;
        pros::Rotation* rotation = nullptr;
        pros::adi::Encoder* adiEncoder = nullptr;
        pros::MotorGroup* motors = nullptr;

        const float diameter;
        const float offset;
        const float gearRatio;
        const float rpm = 0;
};

} // namespace odyssey
