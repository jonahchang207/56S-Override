#pragma once

namespace odyssey {

/**
 * @brief A 2D pose: a position (x, y) and a heading (theta)
 *
 * Used for the robot's position on the field and for motion targets.
 *
 * Units: x and y are in inches. In user-facing code theta is in compass
 * degrees (0 = facing away from the driver station, clockwise positive).
 * Internally the library stores theta in standard math radians
 * (0 = facing +x, counterclockwise positive).
 */
class Pose {
    public:
        /** x position, inches */
        float x;
        /** y position, inches */
        float y;
        /** heading. Compass degrees in user code, math radians internally */
        float theta;

        /**
         * @brief Create a new pose
         *
         * @param x x position, inches
         * @param y y position, inches
         * @param theta heading. Defaults to 0
         */
        Pose(float x = 0, float y = 0, float theta = 0);

        /** add the x and y components of two poses (theta is kept from this pose) */
        Pose operator+(const Pose& other) const;
        /** subtract the x and y components of two poses (theta is kept from this pose) */
        Pose operator-(const Pose& other) const;
        /** 2D dot product of the x and y components */
        float operator*(const Pose& other) const;
        /** scale the x and y components by a scalar */
        Pose operator*(float scalar) const;
        /** divide the x and y components by a scalar */
        Pose operator/(float scalar) const;

        /**
         * @brief Linearly interpolate between this pose and another
         *
         * @param other the pose to interpolate towards
         * @param t interpolation factor. 0 = this pose, 1 = other pose
         */
        Pose lerp(Pose other, float t) const;

        /**
         * @brief Euclidean distance between this pose and another, in inches
         */
        float distance(Pose other) const;

        /**
         * @brief Angle from this pose to another, in standard math radians
         */
        float angle(Pose other) const;

        /**
         * @brief This pose rotated around the origin by an angle
         *
         * @param angle rotation amount, standard math radians
         */
        Pose rotate(float angle) const;
};

} // namespace odyssey
