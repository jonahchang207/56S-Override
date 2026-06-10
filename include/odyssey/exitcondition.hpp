#pragma once

namespace odyssey {

/**
 * @brief Settling detector for motions
 *
 * A motion is considered settled when the error stays inside `range`
 * continuously for `time` milliseconds. Chassis motions use two of these: a
 * tight "small error" condition and a loose "large error" condition, so the
 * robot exits quickly when it is close enough but also gives up gracefully
 * when it stalls just outside the tight range.
 */
class ExitCondition {
    public:
        /**
         * @brief Create a new exit condition
         *
         * @param range the error range where the timer is active
         * @param time how long (ms) the error must stay inside the range
         */
        ExitCondition(float range, int time);

        /**
         * @brief whether the exit condition has been met
         */
        bool getExit() const;

        /**
         * @brief Update with the current error
         *
         * @return whether the exit condition has been met
         */
        bool update(float input);

        /**
         * @brief Reset the timer. Call before starting a new motion
         */
        void reset();

    protected:
        const float range;
        const int time;
        int startTime = -1;
        bool done = false;
};

} // namespace odyssey
