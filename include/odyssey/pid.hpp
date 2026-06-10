#pragma once

namespace odyssey {

/**
 * @brief PID feedback controller
 *
 * Includes integral anti-windup (the integral only accumulates inside a
 * configurable error range) and optional sign-flip reset (the integral is
 * cleared when the error crosses zero, which kills oscillation caused by
 * integral buildup).
 */
class PID {
    public:
        /**
         * @brief Create a new PID controller
         *
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         * @param windupRange integral only accumulates when |error| is below
         *        this value. 0 disables anti-windup
         * @param signFlipReset reset the integral when the error changes sign
         */
        PID(float kP, float kI, float kD, float windupRange = 0, bool signFlipReset = false);

        /**
         * @brief Update the controller with a new error and get the output
         *
         * @param error target - current
         * @return controller output
         */
        float update(float error);

        /**
         * @brief Reset the integral and derivative state. Call this before
         * starting a new movement
         */
        void reset();

        float kP;
        float kI;
        float kD;

    protected:
        const float windupRange;
        const bool signFlipReset;

        float integral = 0;
        float prevError = 0;
};

} // namespace odyssey
