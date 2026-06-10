#pragma once

#include <string>
#include <vector>

#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

#include "odyssey/drivecurve.hpp"
#include "odyssey/exitcondition.hpp"
#include "odyssey/odometry.hpp"
#include "odyssey/pid.hpp"
#include "odyssey/pose.hpp"
#include "odyssey/util.hpp"

namespace odyssey {

/**
 * @brief Physical description of the drivetrain
 */
struct Drivetrain {
        /** motors on the left side of the drivetrain */
        pros::MotorGroup* leftMotors;
        /** motors on the right side of the drivetrain */
        pros::MotorGroup* rightMotors;
        /** distance between the left and right wheels, inches */
        float trackWidth;
        /** diameter of the drive wheels, inches */
        float wheelDiameter;
        /** output rpm of the drive wheels (after external gearing) */
        float rpm;
        /**
         * How much the drivetrain drifts sideways when arcing at speed.
         * Used by moveToPose to limit speed through tight arcs.
         * 2 = all omni wheels (default), 8 = drivetrain with center
         * traction wheels. Higher = less speed restriction
         */
        float horizontalDrift = 2;
};

/**
 * @brief PID gains + settling behavior for a controller (lateral or angular)
 *
 * Error units are inches for the lateral controller and degrees for the
 * angular controller. Outputs are motor powers (-127 to 127).
 */
struct ControllerSettings {
        /** proportional gain */
        float kP;
        /** integral gain */
        float kI;
        /** derivative gain */
        float kD;
        /** integral anti-windup range. 0 disables the integral limit */
        float windupRange;
        /** error range for the fast exit (inches or degrees) */
        float smallError;
        /** time (ms) the error must stay within smallError to exit */
        int smallErrorTimeout;
        /** error range for the slow "give up" exit */
        float largeError;
        /** time (ms) the error must stay within largeError to exit */
        int largeErrorTimeout;
        /** maximum output change per 10ms tick. 0 disables slew */
        float slew;
};

/** parameters for Chassis::turnToHeading */
struct TurnToHeadingParams {
        /** force a turn direction, or AUTO for the shortest path */
        AngularDirection direction = AngularDirection::AUTO;
        /** maximum motor power, 0 to 127 */
        float maxSpeed = 127;
        /** minimum motor power. Use with earlyExitRange for chained motions */
        float minSpeed = 0;
        /** exit (without settling) once the error is below this, degrees */
        float earlyExitRange = 0;
};

/** parameters for Chassis::turnToPoint */
struct TurnToPointParams {
        /** point the front (true) or back (false) of the robot at the target */
        bool forwards = true;
        AngularDirection direction = AngularDirection::AUTO;
        float maxSpeed = 127;
        float minSpeed = 0;
        /** degrees */
        float earlyExitRange = 0;
};

/** parameters for Chassis::swingToHeading */
struct SwingToHeadingParams {
        AngularDirection direction = AngularDirection::AUTO;
        float maxSpeed = 127;
        float minSpeed = 0;
        /** degrees */
        float earlyExitRange = 0;
};

/** parameters for Chassis::moveToPoint */
struct MoveToPointParams {
        /** drive forwards (true) or backwards (false) to the target */
        bool forwards = true;
        float maxSpeed = 127;
        float minSpeed = 0;
        /** exit early once within this distance of the target, inches */
        float earlyExitRange = 0;
};

/** parameters for Chassis::moveToPose */
struct MoveToPoseParams {
        /** drive forwards (true) or backwards (false) into the pose */
        bool forwards = true;
        /**
         * carrot point aggressiveness, 0 to 1. Larger = wider, smoother
         * approach arc; smaller = more direct. 0.6 is a good default
         */
        float lead = 0.6;
        float maxSpeed = 127;
        float minSpeed = 0;
        /** inches */
        float earlyExitRange = 0;
};

/** which side of the drive is locked during a swing turn */
enum class DriveSide { LEFT, RIGHT };

/**
 * @brief A point on a pure pursuit path
 */
struct Waypoint : public Pose {
        Waypoint(float x = 0, float y = 0, float speed = 0)
            : Pose(x, y, 0),
              speed(speed) {}

        /** target speed at this point, -127 to 127 */
        float speed;
};

/**
 * @brief The robot's drivetrain + odometry + motion controller
 *
 * This is the main class of the library. Construct one (usually as a global
 * in main.cpp), call calibrate() in initialize(), and use the motion
 * functions in autonomous() and the drive functions in opcontrol().
 */
class Chassis {
    public:
        /**
         * @brief Create a new chassis
         *
         * @param drivetrain physical drivetrain description
         * @param lateralSettings PID + exit conditions for driving (inches)
         * @param angularSettings PID + exit conditions for turning (degrees)
         * @param sensors odometry sensors
         * @param throttleCurve optional drive curve for the throttle input
         * @param steerCurve optional drive curve for the steering input
         */
        Chassis(Drivetrain drivetrain, ControllerSettings lateralSettings,
                ControllerSettings angularSettings, OdomSensors sensors,
                DriveCurve* throttleCurve = nullptr, DriveCurve* steerCurve = nullptr);

        /**
         * @brief Calibrate the IMU and start odometry. Call in initialize()
         *
         * Takes up to ~3 seconds. Do not move the robot while calibrating.
         * If no vertical tracking wheels were configured, the drive motor
         * encoders are used instead. If the IMU fails to calibrate after 3
         * attempts, odometry falls back to wheel-only heading and the
         * controller rumbles to warn you.
         *
         * @param calibrateImu whether to calibrate the IMU. true by default
         */
        void calibrate(bool calibrateImu = true);

        /**
         * @brief Set the robot's pose. x/y in inches, theta in compass
         * degrees (unless radians = true)
         */
        void setPose(float x, float y, float theta, bool radians = false);
        void setPose(Pose pose, bool radians = false);

        /**
         * @brief Get the robot's pose. theta in compass degrees by default
         */
        Pose getPose(bool radians = false) const;

        // ------------------------------------------------------------------
        // motions
        // ------------------------------------------------------------------

        /**
         * @brief Turn in place to face a heading
         *
         * @param theta target heading, compass degrees
         * @param timeout max time the motion may take, ms
         * @param params optional parameters (direction, speeds, early exit)
         * @param async false to block until the motion finishes
         */
        void turnToHeading(float theta, int timeout, TurnToHeadingParams params = {},
                           bool async = true);

        /**
         * @brief Turn in place to face a point
         *
         * @param x x of the point, inches
         * @param y y of the point, inches
         * @param timeout max time the motion may take, ms
         */
        void turnToPoint(float x, float y, int timeout, TurnToPointParams params = {},
                         bool async = true);

        /**
         * @brief Turn to a heading with one side of the drive locked
         * (an EZ-Template style swing turn)
         *
         * @param theta target heading, compass degrees
         * @param lockedSide which side of the drive stays still
         * @param timeout max time the motion may take, ms
         */
        void swingToHeading(float theta, DriveSide lockedSide, int timeout,
                            SwingToHeadingParams params = {}, bool async = true);

        /**
         * @brief Drive to a point. Heading at arrival is not controlled
         *
         * @param x target x, inches
         * @param y target y, inches
         * @param timeout max time the motion may take, ms
         */
        void moveToPoint(float x, float y, int timeout, MoveToPointParams params = {},
                         bool async = true);

        /**
         * @brief Drive to a pose (position AND final heading) using a
         * boomerang controller
         *
         * @param x target x, inches
         * @param y target y, inches
         * @param theta target heading at arrival, compass degrees
         * @param timeout max time the motion may take, ms
         */
        void moveToPose(float x, float y, float theta, int timeout,
                        MoveToPoseParams params = {}, bool async = true);

        /**
         * @brief Follow a path with the pure pursuit algorithm
         *
         * @param path list of waypoints (x, y, speed)
         * @param lookahead lookahead distance, inches. 10-15 is typical:
         *        smaller = tighter tracking, larger = smoother motion
         * @param timeout max time the motion may take, ms
         * @param forwards drive forwards (true) or backwards (false)
         */
        void follow(const std::vector<Waypoint>& path, float lookahead, int timeout,
                    bool forwards = true, bool async = true);

        /**
         * @brief Follow a path stored on the micro SD card in the
         * path.jerryio format ("x, y, speed" lines, ending with "endData")
         *
         * @param fileName name of the file on the SD card, e.g. "path.txt"
         */
        void follow(const std::string& fileName, float lookahead, int timeout,
                    bool forwards = true, bool async = true);

        // ------------------------------------------------------------------
        // motion management
        // ------------------------------------------------------------------

        /**
         * @brief Block until the current motion has traveled a distance
         *
         * Distance is in inches for drive motions and degrees for turns.
         * Use this to trigger actions (e.g. raising an intake) partway
         * through an async motion.
         */
        void waitUntil(float dist);

        /**
         * @brief Block until the current motion finishes
         */
        void waitUntilDone();

        /**
         * @brief Cancel the current motion. Queued motions still run
         */
        void cancelMotion();

        /**
         * @brief Cancel the current motion and anything queued
         */
        void cancelAllMotions();

        /**
         * @brief whether a motion is currently running
         */
        bool isInMotion() const;

        // ------------------------------------------------------------------
        // driver control
        // ------------------------------------------------------------------

        /**
         * @brief Tank drive: left stick controls the left side, right stick
         * controls the right side
         *
         * @param left left input, -127 to 127
         * @param right right input, -127 to 127
         * @param disableDriveCurve skip the configured drive curves
         */
        void tank(int left, int right, bool disableDriveCurve = false);

        /**
         * @brief Arcade drive: one input for speed, one for turning
         *
         * @param throttle forward input, -127 to 127
         * @param turn turning input, -127 to 127 (positive = right)
         */
        void arcade(int throttle, int turn, bool disableDriveCurve = false);

        /**
         * @brief Curvature drive: like arcade, but the turn input controls
         * the curvature of the arc rather than the turn rate, which feels
         * more natural at speed
         */
        void curvature(int throttle, int turn, bool disableDriveCurve = false);

        /**
         * @brief Set the brake mode of all drive motors
         */
        void setBrakeMode(pros::motor_brake_mode_e mode);

        /** the drivetrain description passed to the constructor */
        Drivetrain drivetrain;
        /** lateral (driving) controller settings */
        ControllerSettings lateralSettings;
        /** angular (turning) controller settings */
        ControllerSettings angularSettings;
        /** odometry sensors */
        OdomSensors sensors;

        /** lateral PID controller, exposed for telemetry / live tuning */
        PID lateralPID;
        /** angular PID controller, exposed for telemetry / live tuning */
        PID angularPID;

    protected:
        /**
         * @brief Take the motion mutex before starting a motion. If another
         * motion is running, the new motion is queued and starts when the
         * running one ends
         */
        void requestMotionStart();

        /**
         * @brief Release the motion mutex when a motion ends
         */
        void endMotion();

        /** stop the drive motors using the current brake mode */
        void stopDrive();

        ExitCondition lateralSmallExit;
        ExitCondition lateralLargeExit;
        ExitCondition angularSmallExit;
        ExitCondition angularLargeExit;

        DriveCurve* throttleCurve;
        DriveCurve* steerCurve;

        /**
         * distance traveled by the current motion (inches for drives,
         * degrees for turns). -1 when no motion is running
         */
        float distTraveled = -1;

        bool motionRunning = false;
        bool motionQueued = false;
        bool calibrated = false;

        pros::Mutex motionMutex;
};

} // namespace odyssey
