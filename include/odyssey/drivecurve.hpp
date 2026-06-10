#pragma once

namespace odyssey {

/**
 * @brief Base class for driver control input curves
 *
 * A drive curve reshapes joystick input (-127 to 127) to make the robot
 * easier to control, typically by making small stick movements gentler
 * while keeping full power available at full deflection.
 */
class DriveCurve {
    public:
        /**
         * @brief Apply the curve to a joystick input
         *
         * @param input joystick value, -127 to 127
         * @return curved output, -127 to 127
         */
        virtual float curve(float input) = 0;

        virtual ~DriveCurve() = default;
};

/**
 * @brief Exponential drive curve
 *
 * output = sign(x) * (minOutput + scaled exponential of |x|)
 *
 * - deadband: inputs smaller than this are ignored (fixes stick drift)
 * - minOutput: minimum output once outside the deadband (overcomes friction)
 * - curve: exponential gain. 1 = linear. Typical values are 1.01 - 1.05;
 *   bigger = gentler around the center, more aggressive near full deflection
 */
class ExpoDriveCurve : public DriveCurve {
    public:
        /**
         * @brief Create a new exponential drive curve
         *
         * @param deadband input deadzone, 0 to 127
         * @param minOutput minimum output when outside the deadband
         * @param curve exponential gain. 1 = linear
         */
        ExpoDriveCurve(float deadband = 0, float minOutput = 0, float curve = 1);

        float curve(float input) override;

    protected:
        const float deadband;
        const float minOutput;
        const float curveGain;
};

} // namespace odyssey
