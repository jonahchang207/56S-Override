#pragma once

/**
 * @file chassis.hpp
 * @brief Motion-control primitives and the Chassis driver class.
 *
 * Provides everything a typical PROS autonomous needs:
 *   - Low-level helpers: PID, Timer, SlewLimiter, ExitCondition, ExpoDriveCurve.
 *   - Parameter structs for each high-level motion (turn / swing / move / follow).
 *   - SafetyConfig + MotorTemperature for fault detection and telemetry.
 *   - Path utilities: inject_path, smooth_path, load_path.
 *   - MotionConfig: the bundle of PID gains and tuning constants used by the
 *     Chassis.
 *   - Chassis: the main driver class. Tank/arcade for opcontrol, a family of
 *     blocking + async methods for autonomous, and a safety/telemetry layer.
 */

#include "api.h"
#include "odom.hpp"
#include "robot_config.hpp"

#include <cstdint>
#include <initializer_list>
#include <string>
#include <vector>

namespace Vortex {

/**
 * @struct PIDGains
 * @brief Tuning constants for a single PID loop.
 *
 * @c integral_zone keeps the I term off until error is below the threshold,
 * which prevents windup on the first swing toward a target. @c max_integral
 * clamps the accumulated I term so it cannot push the output past
 * controllable bounds.
 */
struct PIDGains {
  double kP = 0.0;             ///< Proportional gain.
  double kI = 0.0;             ///< Integral gain.
  double kD = 0.0;             ///< Derivative gain.
  double integral_zone = 0.0;  ///< Only integrate when |error| < this value.
  double max_integral = 0.0;   ///< Hard cap on the integral term.
};

/**
 * @class PID
 * @brief Generic discrete PID controller.
 */
class PID {
 public:
  /** @brief Construct with all gains zero. */
  PID() = default;

  /** @brief Construct with the given gains. */
  explicit PID(const PIDGains& gains);

  /** @brief Swap in new gains. State (integral, last error) is preserved. */
  void set_gains(const PIDGains& gains);

  /** @brief Read back the current gains. */
  PIDGains get_gains() const;

  /** @brief Clear the integral, derivative history, and "first step" flag. */
  void reset();

  /**
   * @brief Run one PID step and return the output.
   * @param error Target - measurement.
   * @return Raw controller output (unclamped).
   */
  double step(double error);

 private:
  PIDGains gains_;
  double integral_ = 0.0;
  double last_error_ = 0.0;
  bool first_step_ = true;
};

/**
 * @class Timer
 * @brief Thin wrapper around pros::millis() for elapsed-time queries.
 */
class Timer {
 public:
  /** @brief Construct and start the timer. */
  Timer();

  /** @brief Restart the timer from now. */
  void reset();

  /** @brief Milliseconds since reset() / construction. */
  std::uint32_t elapsed_ms() const;

  /** @brief True when elapsed_ms() >= @p duration_ms. */
  bool done(std::uint32_t duration_ms) const;

 private:
  std::uint32_t start_ms_ = 0;
};

/**
 * @class SlewLimiter
 * @brief Limits how fast an output value may change per call.
 *
 * Useful for protecting drive motors from instantaneous voltage jumps that
 * cause wheel slip or current spikes.
 */
class SlewLimiter {
 public:
  /** @brief Construct with the maximum allowed delta per step. */
  explicit SlewLimiter(double step = 12000.0);

  /** @brief Update the maximum delta. */
  void set_step(double step);

  /** @brief Snap the internal value to @p value (default 0). */
  void reset(double value = 0.0);

  /**
   * @brief Advance toward @p target by at most one step.
   * @return The (possibly limited) new output.
   */
  double step(double target);

 private:
  double step_ = 12000.0;
  double value_ = 0.0;
};

/**
 * @struct ExitCondition
 * @brief Two-tier "are we there yet?" check based on error magnitude.
 *
 * The "small" tier triggers when the robot is comfortably inside the target;
 * the "big" tier exits early if the robot has been close enough for a long
 * time. Both tiers must remain satisfied for their respective hold times.
 */
struct ExitCondition {
  double small_error = 1.0;             ///< Tight error threshold.
  std::uint32_t small_time_ms = 250;    ///< Hold time at small_error.
  double big_error = 3.0;               ///< Loose error threshold.
  std::uint32_t big_time_ms = 500;      ///< Hold time at big_error.

  /** @brief Clear the internal timers. */
  void reset();

  /**
   * @brief Update the condition with the latest |error|.
   * @param error_abs Absolute value of the current error.
   * @return True when either tier has been satisfied for long enough.
   */
  bool settled(double error_abs);

 private:
  std::uint32_t small_since_ = 0;
  std::uint32_t big_since_ = 0;
};

/**
 * @struct ExpoDriveCurve
 * @brief Driver-feel curve applied to joystick input in opcontrol.
 *
 * Sub-deadband inputs map to zero; otherwise the absolute input is raised
 * to @c curve power and scaled back into [min_output, 127].
 */
struct ExpoDriveCurve {
  int deadband = 5;     ///< Dead-zone around 0.
  int min_output = 0;   ///< Minimum nonzero output magnitude.
  double curve = 1.6;   ///< Exponent applied to |input|.

  /**
   * @brief Apply the curve to a joystick value.
   * @param input Raw joystick value, expected in [-127, 127].
   * @return Curved output in [-127, 127].
   */
  int apply(int input) const;
};

/**
 * @struct TrapezoidProfileConfig
 * @brief Acceleration / deceleration shaping for motion commands.
 */
struct TrapezoidProfileConfig {
  bool enabled = true;             ///< Master switch.

  double linear_accel_in = 8.0;    ///< Ramp-up window for linear moves (in).
  double linear_decel_in = 10.0;   ///< Ramp-down window for linear moves (in).

  double angular_accel_deg = 25.0; ///< Ramp-up window for turns/swings (deg).
  double angular_decel_deg = 35.0; ///< Ramp-down window for turns/swings (deg).

  int min_speed = 8;               ///< Default floor when none is supplied.
};

/**
 * @enum AngularDirection
 * @brief Direction hint for turn-style motions.
 */
enum class AngularDirection {
  AUTO,                  ///< Pick whichever direction has shorter error.
  CW_CLOCKWISE,          ///< Force clockwise (positive theta).
  CCW_COUNTERCLOCKWISE,  ///< Force counter-clockwise (negative theta).
};

/**
 * @enum DriveSide
 * @brief Which side stays locked during a swing.
 */
enum class DriveSide {
  LEFT,   ///< Left side held; right side drives the swing.
  RIGHT,  ///< Right side held; left side drives the swing.
};

/** @brief Parameter bundle for Chassis::turnToHeading. */
struct TurnToHeadingParams {
  AngularDirection direction = AngularDirection::AUTO;  ///< Direction hint.
  int max_speed = 127;                                  ///< Max voltage % (0-127).
  int min_speed = 0;                                    ///< Minimum nonzero output.
  double early_exit_range_deg = 0.0;                    ///< Exit when |error| < this.
};

/** @brief Parameter bundle for Chassis::turnToPoint. */
struct TurnToPointParams {
  bool forwards = true;                                 ///< Face point with front (true) or back.
  AngularDirection direction = AngularDirection::AUTO;
  int max_speed = 127;
  int min_speed = 0;
  double early_exit_range_deg = 0.0;
};

/** @brief Parameter bundle for Chassis::swingToHeading. */
struct SwingToHeadingParams {
  AngularDirection direction = AngularDirection::AUTO;
  int max_speed = 127;
  int min_speed = 0;
  double early_exit_range_deg = 0.0;
};

/** @brief Parameter bundle for Chassis::swingToPoint. */
struct SwingToPointParams {
  bool forwards = true;
  AngularDirection direction = AngularDirection::AUTO;
  int max_speed = 127;
  int min_speed = 0;
  double early_exit_range_deg = 0.0;
};

/** @brief Parameter bundle for Chassis::moveToPoint. */
struct MoveToPointParams {
  bool forwards = true;             ///< Drive forward (true) or reverse.
  int max_speed = 127;
  int min_speed = 0;
  double early_exit_range_in = 0.0; ///< Exit when distance-to-target < this.
  bool slew = false;                ///< Apply slew rate to drive voltage.
};

/** @brief Parameter bundle for Chassis::moveToPose. */
struct MoveToPoseParams {
  bool forwards = true;
  int max_speed = 127;
  int min_speed = 0;
  double early_exit_range_in = 0.0;
  double early_exit_heading_deg = 0.0;
  double lead = 0.6;          ///< Boomerang lead factor toward final heading.
  double max_lead_in = 24.0;  ///< Max projected lead distance.
  double turn_weight = 0.45;  ///< Trade-off between turning and translating.
  bool slew = false;
};

/** @brief Parameter bundle for Chassis::follow. */
struct FollowParams {
  bool forwards = true;
  int max_speed = 127;
  int min_speed = 20;
  double lookahead_in = 12.0;        ///< Pure-pursuit lookahead radius.
  double early_exit_range_in = 2.0;
  bool smooth = false;               ///< Apply smooth_path() before following.
  double inject_spacing_in = 3.0;    ///< inject_path() spacing.
  double smooth_weight_data = 0.75;
  double smooth_weight_smooth = 0.25;
};

/**
 * @struct SafetyConfig
 * @brief Hardware-fault response policy for the Chassis.
 */
struct SafetyConfig {
  bool stop_on_over_current = true;     ///< Bail if a side over-currents.
  bool stop_on_over_temp = true;        ///< Bail if a motor over-heats.
  bool stop_on_stall = true;            ///< Bail if a side stalls.
  double stall_velocity_rpm = 3.0;      ///< Below this is "not moving".
  int stall_current_ma = 1800;          ///< Above this with no motion = stall.
  std::uint32_t stall_time_ms = 250;    ///< Stall must persist this long.
};

/**
 * @struct MotorTemperature
 * @brief One row of motor telemetry exposed by Chassis::motorTemperatures().
 */
struct MotorTemperature {
  std::string tag;     ///< Side identifier, e.g. "L" or "R".
  int port = 0;        ///< 1-indexed V5 port, sign matches motor reversal.
  double celsius = 0.0;///< Reported temperature.
  int percent = 0;     ///< Reported "thermal headroom" percent.
};

/**
 * @struct PathPoint
 * @brief One waypoint in a Path: (x, y) field point + target speed.
 */
struct PathPoint {
  double x = 0.0;    ///< x in inches.
  double y = 0.0;    ///< y in inches.
  int speed = 127;   ///< Target speed at this waypoint (0-127).

  PathPoint() = default;

  /**
   * @param ix      x in inches.
   * @param iy      y in inches.
   * @param ispeed  Target speed at this waypoint (0-127).
   */
  PathPoint(double ix, double iy, int ispeed = 127)
      : x(ix), y(iy), speed(ispeed) {}
};

/** @brief Sequence of waypoints describing a desired path. */
typedef std::vector<PathPoint> Path;

/**
 * @brief Build a path from a list of waypoints (EZ-Template-style list init).
 *
 * @code
 * const Vortex::Path path = Vortex::make_path({
 *     {0.0, 0.0, 100},
 *     {0.0, 24.0, 110},
 *     {24.0, 24.0, 90},
 * });
 * @endcode
 */
Path make_path(std::initializer_list<PathPoint> points);

/**
 * @brief Insert intermediate waypoints so consecutive points are no farther
 *        apart than @p spacing_in inches.
 */
Path inject_path(const Path& path, double spacing_in);

/**
 * @brief Iteratively smooth a path (gradient-descent style).
 * @param weight_data    Pull toward original points.
 * @param weight_smooth  Pull toward neighbours (lower curvature).
 * @param tolerance      Stopping threshold for total point movement.
 */
Path smooth_path(const Path& path, double weight_data = 0.75,
                 double weight_smooth = 0.25, double tolerance = 0.001);

/**
 * @brief Load a Path from a file on the SD card.
 *
 * Expected format is one comma-separated record per line:
 * @verbatim x,y,speed @endverbatim
 */
Path load_path(const char* filename);

/**
 * @struct MotionConfig
 * @brief Every PID + tuning constant the Chassis needs in one place.
 *
 * The defaults are a reasonable starting point for a typical 4-motor
 * drive with tracking wheels. Use PIDTuner in opcontrol to dial them in.
 */
struct MotionConfig {
  PIDGains drive = {650.0, 0.0, 1800.0, 3.0, 2500.0};         ///< drive_distance.
  PIDGains turn = {90.0, 0.0, 650.0, 8.0, 800.0};             ///< turnToHeading.
  PIDGains swing = {95.0, 0.0, 700.0, 8.0, 800.0};            ///< swingTo*.
  PIDGains point_drive = {520.0, 0.0, 1200.0, 3.0, 2200.0};   ///< moveToPoint drive.
  PIDGains point_turn = {75.0, 0.0, 480.0, 8.0, 650.0};       ///< moveToPoint turn.
  PIDGains pose_drive = {500.0, 0.0, 1150.0, 3.0, 2200.0};    ///< moveToPose drive.
  PIDGains pose_turn = {70.0, 0.0, 500.0, 8.0, 650.0};        ///< moveToPose turn.
  PIDGains follow_drive = {450.0, 0.0, 900.0, 3.0, 1600.0};   ///< follow() drive.
  PIDGains follow_turn = {65.0, 0.0, 380.0, 8.0, 500.0};      ///< follow() turn.

  double drive_settle_error_in = 0.75;       ///< Linear settle window (in).
  double turn_settle_error_deg = 1.5;        ///< Angular settle window (deg).
  std::uint32_t settle_time_ms = 250;        ///< Hold time before declaring done.
  std::uint32_t loop_delay_ms = 10;          ///< Control loop period.
  double slew_step_mv = 700.0;               ///< Voltage slew rate when enabled.
  TrapezoidProfileConfig trapezoid;          ///< Motion-profile shaping.
};

/**
 * @class Chassis
 * @brief High-level driver class for a differential drive base.
 *
 * Owns motor groups, an Odometry pointer, PID controllers, drive curves,
 * a safety layer, and an async motion task. Mirrors common LemLib-style
 * names (turnToHeading, moveToPoint, moveToPose, follow) so existing
 * autonomous routines port easily.
 *
 * Construction patterns:
 * @code
 * // Minimum required (no telemetry of individual motors).
 * Vortex::Chassis chassis(&left_drive, &right_drive, &odom);
 *
 * // Pass port lists too so motorTemperatures() can label per-motor data.
 * Vortex::Chassis chassis(&left_drive, &right_drive, &odom,
 *                         {4, -5, -6}, {1, -2, 3});
 * @endcode
 */
class Chassis {
 public:
  /**
   * @brief Construct with motor groups and an Odometry source.
   */
  Chassis(pros::MotorGroup* left, pros::MotorGroup* right, Odometry* odom);

  /**
   * @brief Construct and provide port lists for per-motor telemetry.
   * @param left         Left drive motor group.
   * @param right        Right drive motor group.
   * @param odom         Odometry pose source.
   * @param left_ports   V5 ports for the left side (sign = reversal).
   * @param right_ports  V5 ports for the right side (sign = reversal).
   */
  Chassis(pros::MotorGroup* left, pros::MotorGroup* right, Odometry* odom,
          const std::vector<int>& left_ports,
          const std::vector<int>& right_ports);

  /**
   * @brief Construct with a @ref DrivetrainConfig port list (LemLib-style).
   */
  Chassis(pros::MotorGroup* left, pros::MotorGroup* right, Odometry* odom,
          const DrivetrainConfig& drivetrain);

  /**
   * @brief Default driver setup: coast brake, standard expo curves.
   * Call once in @c initialize() after odometry is ready.
   */
  void init(pros::motor_brake_mode_e_t brake_mode = pros::E_MOTOR_BRAKE_COAST);

  /** @brief Replace the motion configuration in one shot. */
  void set_motion_config(const MotionConfig& config);

  /** @brief Snapshot of the current motion configuration. */
  MotionConfig get_motion_config() const;

  /** @brief Pointer to the live motion config (for PIDTuner-style editing). */
  MotionConfig* motion_config();

  /** @brief Replace the safety configuration. */
  void set_safety_config(const SafetyConfig& config);

  /** @brief Snapshot of the current safety configuration. */
  SafetyConfig get_safety_config() const;

  /** @brief Most recent safety reason string, e.g. "stall_right". */
  const char* last_safety_message() const;

  /** @brief Replace the drive motor groups. */
  void set_drive_groups(pros::MotorGroup* left, pros::MotorGroup* right);

  /** @brief Replace the per-motor port lists used for telemetry. */
  void set_drive_motor_ports(const std::vector<int>& left_ports,
                             const std::vector<int>& right_ports);

  /** @brief Install new throttle / turn curves used by arcade() and tank(). */
  void set_drive_curves(const ExpoDriveCurve& throttle,
                        const ExpoDriveCurve& turn);

  /** @brief Apply a PROS brake mode to both drive sides. */
  void set_brake_mode(pros::motor_brake_mode_e_t mode);

  /** @brief Send raw millivolts to each side (no curves, no clamping shape). */
  void tank_voltage(int left_mv, int right_mv);

  /**
   * @brief Tank-style driver control.
   * @param left                 Left joystick value [-127, 127].
   * @param right                Right joystick value [-127, 127].
   * @param disable_drive_curve  Skip the expo curves if true.
   */
  void tank(int left, int right, bool disable_drive_curve = false);

  /**
   * @brief Arcade-style driver control.
   * @param throttle             Forward/backward axis [-127, 127].
   * @param turn                 Left/right axis [-127, 127].
   * @param disable_drive_curve  Skip the expo curves if true.
   * @param desaturate_bias      0 = clip turn first, 1 = clip throttle first.
   */
  void arcade(int throttle, int turn, bool disable_drive_curve = false,
              double desaturate_bias = 0.5);

  /**
   * @brief Curvature-style driver control (turn scales with throttle).
   */
  void curvature(int throttle, int turn, bool disable_drive_curve = false);

  /** @brief Stop both sides using the current brake mode. */
  void brake();

  /**
   * @brief Blocking forward / backward move by relative distance.
   * @param distance_in  Signed distance in inches (negative = reverse).
   * @param timeout_ms   Hard timeout.
   * @param max_voltage  Voltage cap in millivolts.
   * @return True if the motion settled before the timeout.
   */
  bool drive_distance(double distance_in, std::uint32_t timeout_ms = 3000,
                      int max_voltage = 12000);

  /**
   * @brief Blocking turn to an absolute heading.
   * @param heading_deg  Target heading in degrees.
   */
  bool turn_to_heading(double heading_deg, std::uint32_t timeout_ms = 2000,
                       int max_voltage = 12000);

  /**
   * @brief Blocking turn to face a field point.
   */
  bool turn_to_point(double x_in, double y_in, std::uint32_t timeout_ms = 2000,
                     int max_voltage = 12000);

  /**
   * @brief Blocking drive-to-point (no heading control at the goal).
   */
  bool drive_to_point(double x_in, double y_in, std::uint32_t timeout_ms = 4000,
                      int max_voltage = 11000);

  /**
   * @brief Blocking drive to a target pose (x, y, heading).
   */
  bool drive_to_pose(double x_in, double y_in, double heading_deg,
                     std::uint32_t timeout_ms = 4000, int max_voltage = 11000,
                     const MoveToPoseParams& params = MoveToPoseParams());

  /**
   * @brief Blocking pure-pursuit path follow with sensible defaults.
   */
  bool follow_path(const Path& path, std::uint32_t timeout_ms = 5000,
                   const FollowParams& params = FollowParams());

  /**
   * @brief LemLib-style turnToHeading; blocking when @p async is false.
   */
  void turnToHeading(double theta_deg, int timeout_ms,
                     TurnToHeadingParams params = TurnToHeadingParams(),
                     bool async = true);

  /**
   * @brief LemLib-style turnToPoint; blocking when @p async is false.
   */
  void turnToPoint(double x_in, double y_in, int timeout_ms,
                   TurnToPointParams params = TurnToPointParams(),
                   bool async = true);

  /**
   * @brief Pivot one side around the other to reach @p theta_deg.
   * @param locked_side Side that stays braked.
   */
  void swingToHeading(double theta_deg, DriveSide locked_side, int timeout_ms,
                      SwingToHeadingParams params = SwingToHeadingParams(),
                      bool async = true);

  /**
   * @brief Swing to face a field point with one side locked.
   */
  void swingToPoint(double x_in, double y_in, DriveSide locked_side,
                    int timeout_ms,
                    SwingToPointParams params = SwingToPointParams(),
                    bool async = true);

  /**
   * @brief Drive to (x, y) with concurrent heading control.
   */
  void moveToPoint(double x_in, double y_in, int timeout_ms,
                   MoveToPointParams params = MoveToPointParams(),
                   bool async = true);

  /**
   * @brief Drive to a target pose (x, y, heading) with a boomerang lead.
   */
  void moveToPose(double x_in, double y_in, double theta_deg, int timeout_ms,
                  MoveToPoseParams params = MoveToPoseParams(),
                  bool async = true);

  /**
   * @brief Follow a Path using pure-pursuit-style control.
   */
  void follow(const Path& path, int timeout_ms,
              FollowParams params = FollowParams(), bool async = true);

  /** @brief Cancel any currently running async motion. */
  void cancelMotion();

  /**
   * @brief Block until motion_progress() reaches @p progress (0-1).
   *
   * Useful for triggering subsystem actions partway through a motion.
   */
  void waitUntil(double progress);

  /** @brief Block until the current async motion finishes. */
  void waitUntilDone();

  /** @brief True while an async motion is executing. */
  bool isMotionRunning() const;

  /** @brief True iff the last motion finished by settling (not by timeout). */
  bool lastMotionSucceeded() const;

  /** @brief Progress through the current motion, 0 .. 1. */
  double motionProgress() const;

  /**
   * @struct MotionTarget
   * @brief Snapshot of the current motion goal for telemetry / dashboards.
   */
  struct MotionTarget {
    bool valid = false;        ///< False when no motion is active.
    const char* type = "Idle"; ///< Short string label.
    double x = 0.0;            ///< Target x (inches), if applicable.
    double y = 0.0;            ///< Target y (inches), if applicable.
    double theta = 0.0;        ///< Target heading (deg), if applicable.
    double distance = 0.0;     ///< Target distance (inches), if applicable.
  };

  /** @brief Snapshot of what the chassis is currently doing. */
  MotionTarget currentTarget() const;

  /** @brief One row per drive motor with temperature + headroom percent. */
  std::vector<MotorTemperature> motorTemperatures() const;

 private:
  enum class MotionType {
    NONE,
    TURN_HEADING,
    TURN_POINT,
    SWING_HEADING,
    SWING_POINT,
    MOVE_POINT,
    MOVE_POSE,
    FOLLOW,
  };

  struct MotionCommand {
    MotionType type = MotionType::NONE;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    int timeout_ms = 0;
    DriveSide locked_side = DriveSide::LEFT;
    TurnToHeadingParams turn_heading;
    TurnToPointParams turn_point;
    SwingToHeadingParams swing_heading;
    SwingToPointParams swing_point;
    MoveToPointParams move_point;
    MoveToPoseParams move_pose;
    FollowParams follow_params;
    Path path;
  };

  static void task_entry(void* params);
  void dispatch(const MotionCommand& command, bool async);
  void start_async(const MotionCommand& command);
  bool run_command(const MotionCommand& command);

  bool run_drive_distance(double distance_in, std::uint32_t timeout_ms,
                          int max_voltage);
  bool run_turn_to_heading(double heading_deg, int timeout_ms,
                           const TurnToHeadingParams& params);
  bool run_swing_to_heading(double heading_deg, DriveSide locked_side,
                            int timeout_ms,
                            const SwingToHeadingParams& params);
  bool run_move_to_point(double x_in, double y_in, int timeout_ms,
                         const MoveToPointParams& params);
  bool run_move_to_pose(double x_in, double y_in, double theta_deg,
                        int timeout_ms, const MoveToPoseParams& params);
  bool run_follow(Path path, int timeout_ms, const FollowParams& params);

  int clamp_voltage(double voltage, int max_voltage) const;
  int speed_to_voltage(int speed) const;
  int voltage_to_speed(int voltage) const;
  int apply_min_speed(int voltage, int min_speed) const;
  int trapezoid_limit(double progress, double remaining, int max_voltage,
                      int min_speed, double accel_distance,
                      double decel_distance) const;
  int apply_trapezoid(double voltage, double progress, double remaining,
                      int max_voltage, int min_speed, double accel_distance,
                      double decel_distance) const;
  void tank_drive_turn_profiled(int drive_mv, int turn_mv,
                                int side_limit_mv);
  bool timed_out(std::uint32_t start_ms, std::uint32_t timeout_ms) const;
  bool safety_ok(int commanded_left_mv, int commanded_right_mv);
  bool group_over_current(pros::MotorGroup* group) const;
  bool group_over_temp(pros::MotorGroup* group) const;
  bool group_stalled(pros::MotorGroup* group, int command_mv) const;
  void set_target(const char* type, double x = 0.0, double y = 0.0,
                  double theta = 0.0, double distance = 0.0);
  double directed_error(double target_deg, double current_deg,
                        AngularDirection direction) const;
  double interpolate_heading(double from_deg, double to_deg, double t) const;

  pros::MotorGroup* left_ = nullptr;
  pros::MotorGroup* right_ = nullptr;
  Odometry* odom_ = nullptr;
  MotionConfig config_;
  SafetyConfig safety_config_;
  const char* safety_message_ = "ok";
  std::uint32_t stall_since_ms_ = 0;
  std::vector<int> left_drive_ports_;
  std::vector<int> right_drive_ports_;
  MotionTarget current_target_;
  ExpoDriveCurve throttle_curve_;
  ExpoDriveCurve turn_curve_;

  PID drive_pid_;
  PID turn_pid_;
  PID swing_pid_;
  PID point_drive_pid_;
  PID point_turn_pid_;
  PID pose_drive_pid_;
  PID pose_turn_pid_;
  PID follow_drive_pid_;
  PID follow_turn_pid_;

  MotionCommand command_;
  pros::Task* motion_task_ = nullptr;
  bool motion_running_ = false;
  bool motion_cancel_ = false;
  bool motion_success_ = false;
  double motion_progress_ = 0.0;
};

}  // namespace Vortex
