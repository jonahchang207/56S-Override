#pragma once

/**
 * @file odom.hpp
 * @brief Pose tracking (odometry) for a VEX V5 robot.
 *
 * This file defines the Vortex odometry stack:
 *   - Pose: an (x, y, heading) snapshot of the robot on the field.
 *   - DistanceSensor: an abstract source of linear inches traveled.
 *   - RotationDistanceSensor / MotorGroupDistanceSensor: concrete sensors
 *     built on top of @c pros::Rotation and @c pros::MotorGroup.
 *   - TrackingWheel: pairs a DistanceSensor with its offset from robot center.
 *   - OdomConfig: a builder/struct that bundles up every input the fusion
 *     loop needs.
 *   - Odometry: the runtime object that owns the update task and exposes
 *     get_pose() / set_pose() to the rest of the program.
 */

#include "api.h"
#include "robot_config.hpp"

#include <cstdint>
#include <memory>

/**
 * @namespace Vortex
 * @brief All public symbols of the Vortex odometry / motion library.
 */
namespace Vortex {

/** @brief Pi to double precision. */
constexpr double kPi = 3.14159265358979323846;

/**
 * @brief Convert degrees to radians.
 * @param deg Angle in degrees.
 * @return Angle in radians.
 */
double deg_to_rad(double deg);

/**
 * @brief Convert radians to degrees.
 * @param rad Angle in radians.
 * @return Angle in degrees.
 */
double rad_to_deg(double rad);

/**
 * @brief Wrap a degree angle into the range (-180, 180].
 * @param deg Angle in degrees.
 * @return Equivalent angle in (-180, 180].
 */
double wrap_deg(double deg);

/**
 * @brief Wrap a radian angle into the range (-pi, pi].
 * @param rad Angle in radians.
 * @return Equivalent angle in (-pi, pi].
 */
double wrap_rad(double rad);

/**
 * @brief Shortest signed error between two headings.
 * @param target_deg  Desired heading in degrees.
 * @param current_deg Current heading in degrees.
 * @return Signed error (target - current) wrapped to (-180, 180].
 */
double angle_error_deg(double target_deg, double current_deg);

/**
 * @brief Euclidean distance between two field points.
 * @return @f$\sqrt{(x_2-x_1)^2 + (y_2-y_1)^2}@f$ in inches.
 */
double distance_between(double x1, double y1, double x2, double y2);

/**
 * @brief Field-frame bearing from one point to another, in degrees.
 *
 * The result is clockwise from the +Y (field forward) axis, matching the
 * convention used by Pose::heading_deg().
 */
double bearing_to_point_deg(double from_x, double from_y, double to_x,
                            double to_y);

/**
 * @struct Pose
 * @brief A snapshot of robot position and heading on the field.
 *
 * Coordinates are inches. Heading is stored internally in radians but
 * helpers like Pose::from_degrees and Pose::set_heading_deg let you work in
 * degrees if that is more natural.
 *
 * Convention:
 *  - +x: field right (positive to driver right)
 *  - +y: field forward (away from driver station)
 *  - theta: clockwise from +Y, so 90 deg means facing field-right.
 */
struct Pose {
  double x = 0.0;      ///< Inches, positive to field right.
  double y = 0.0;      ///< Inches, positive away from the driver station.
  double theta = 0.0;  ///< Radians, clockwise from field +Y.

  /** @brief Construct a zero pose at (0, 0) facing 0 rad. */
  Pose() = default;

  /**
   * @brief Construct a pose from x, y and a heading in radians.
   * @param ix          x position in inches.
   * @param iy          y position in inches.
   * @param itheta_rad  Heading in radians.
   */
  Pose(double ix, double iy, double itheta_rad)
      : x(ix), y(iy), theta(itheta_rad) {}

  /**
   * @brief Factory that builds a Pose from a heading in degrees.
   * @param x_in        x position in inches.
   * @param y_in        y position in inches.
   * @param heading_deg Heading in degrees, clockwise from +Y.
   */
  static Pose from_degrees(double x_in, double y_in, double heading_deg);

  /** @brief Current heading converted to degrees. */
  double heading_deg() const;

  /**
   * @brief Replace the heading using a degree value.
   * @param heading_deg Heading in degrees.
   */
  void set_heading_deg(double heading_deg);
};

/**
 * @class DistanceSensor
 * @brief Abstract source of linear inches traveled.
 *
 * Any object that can answer "how many inches have I moved since the last
 * reset?" can plug into the odometry. Implement this interface to add a new
 * encoder type (e.g. an ADI quad encoder).
 */
class DistanceSensor {
 public:
  virtual ~DistanceSensor() = default;

  /** @brief Inches traveled since the last reset(). */
  virtual double get_distance_in() const = 0;

  /** @brief Zero the sensor and apply any cached configuration. */
  virtual void reset() = 0;
};

/**
 * @class RotationDistanceSensor
 * @brief DistanceSensor implementation backed by a @c pros::Rotation sensor.
 *
 * Reads the rotation sensor in centidegrees, converts to wheel revolutions
 * using the supplied gear ratio, then multiplies by wheel circumference to
 * return inches.
 */
class RotationDistanceSensor : public DistanceSensor {
 public:
  /**
   * @param sensor             The PROS rotation sensor (may be nullptr).
   * @param wheel_diameter_in  Tracking wheel diameter in inches.
   * @param gear_ratio         Sensor revolutions per wheel revolution.
   *                           Use 1.0 if the wheel is directly on the sensor.
   * @param reversed           Flip direction if forward motion reads negative.
   */
  RotationDistanceSensor(pros::Rotation* sensor, double wheel_diameter_in,
                         double gear_ratio = 1.0, bool reversed = false);

  double get_distance_in() const override;
  void reset() override;

 private:
  pros::Rotation* sensor_ = nullptr;
  double wheel_diameter_in_ = 0.0;
  double gear_ratio_ = 1.0;
  bool reversed_ = false;
  mutable double offset_degrees_ = 0.0;
};

/**
 * @class MotorGroupDistanceSensor
 * @brief DistanceSensor implementation that averages a drive motor group.
 *
 * Use this as a fallback when no dedicated tracking wheel exists; it reads
 * the average rotor position of the supplied @c pros::MotorGroup.
 */
class MotorGroupDistanceSensor : public DistanceSensor {
 public:
  /**
   * @param motors                   Drive motor group to sample.
   * @param wheel_diameter_in        Drive wheel diameter in inches.
   * @param wheel_rev_per_motor_rev  Wheel revolutions per motor revolution
   *                                 (gear ratio output / input).
   * @param reversed                 Flip direction if forward motion reads
   *                                 negative.
   */
  MotorGroupDistanceSensor(pros::MotorGroup* motors, double wheel_diameter_in,
                           double wheel_rev_per_motor_rev = 1.0,
                           bool reversed = false);

  double get_distance_in() const override;
  void reset() override;

 private:
  pros::MotorGroup* motors_ = nullptr;
  double wheel_diameter_in_ = 0.0;
  double wheel_rev_per_motor_rev_ = 1.0;
  int direction_ = 1;
  mutable double offset_degrees_ = 0.0;
};

/**
 * @class Wheel
 * @brief LemLib-style tracking wheel: port, diameter, offset in one object.
 *
 * Owns a @c pros::Rotation (rotation variant) or references a drive
 * @c pros::MotorGroup (motor variant) and exposes a @ref DistanceSensor for
 * the fusion loop.
 *
 * @code
 * Vortex::Wheel vertical(-18, 2.0, 0.0);   // port, diameter in, offset in
 * Vortex::Wheel horizontal(19, 2.75, -4.75, 1.0, true);  // optional reverse
 * @endcode
 */
class Wheel {
 public:
  /**
   * @brief Rotation-sensor tracking wheel.
   * @param rotation_port      PROS rotation port (negative reverses the sensor).
   * @param wheel_diameter_in  Wheel diameter in inches.
   * @param offset_in          Signed offset from the robot tracking center.
   * @param gear_ratio         Sensor revolutions per wheel revolution.
   * @param reversed           Flip direction if forward motion reads negative.
   */
  Wheel(int rotation_port, double wheel_diameter_in, double offset_in,
        double gear_ratio = 1.0, bool reversed = false);

  /**
   * @brief Drive-motor tracking wheel (EZ-Template-style motor encoder).
   * @param motors             Existing drive motor group.
   * @param wheel_diameter_in  Wheel diameter in inches.
   * @param offset_in          Signed offset from the robot tracking center.
   * @param gear_ratio         Wheel revolutions per motor revolution.
   * @param reversed           Flip direction if forward motion reads negative.
   */
  Wheel(pros::MotorGroup* motors, double wheel_diameter_in, double offset_in,
        double gear_ratio = 1.0, bool reversed = false);

  /** @brief Build from a @ref TrackerConfig row in @c Config.cpp. */
  explicit Wheel(const TrackerConfig& config);

  Wheel(const Wheel&) = delete;
  Wheel& operator=(const Wheel&) = delete;
  Wheel(Wheel&&) = default;
  Wheel& operator=(Wheel&&) = default;

  /** @brief Distance source wired into odometry. */
  DistanceSensor* sensor() const;

  /** @brief Signed mounting offset in inches. */
  double offset_in() const;

  /** @brief Underlying rotation sensor, or nullptr for motor wheels. */
  pros::Rotation* rotation() const;

 private:
  double offset_in_ = 0.0;
  std::unique_ptr<pros::Rotation> rotation_;
  std::unique_ptr<RotationDistanceSensor> rotation_distance_;
  std::unique_ptr<MotorGroupDistanceSensor> motor_distance_;
  DistanceSensor* sensor_ = nullptr;
};

/**
 * @struct OdomSensors
 * @brief LemLib-style bundle of every sensor the pose estimator needs.
 *
 * Pass to @ref Odometry's constructor, then call @ref Odometry::init() once in
 * @c initialize().
 *
 * @code
 * Vortex::Wheel vertical(-18, 2.0, 0.0);
 * Vortex::Wheel horizontal(19, 2.75, -4.75);
 * Vortex::Odometry odom(Vortex::OdomSensors(&imu)
 *                           .vertical(&vertical)
 *                           .horizontal(&horizontal));
 *
 * void initialize() {
 *   odom.init();  // calibrate IMU, zero encoders, set (0,0,0), start task
 * }
 * @endcode
 */
struct OdomSensors {
  pros::Imu* imu = nullptr;
  bool use_imu = true;

  Wheel* vertical = nullptr;
  Wheel* vertical2 = nullptr;
  Wheel* horizontal = nullptr;
  Wheel* horizontal2 = nullptr;

  pros::MotorGroup* left_drive = nullptr;
  pros::MotorGroup* right_drive = nullptr;
  double drive_track_width_in = 0.0;
  double drive_wheel_diameter_in = 3.25;
  double drive_gear_ratio = 1.0;
  bool drive_reversed = false;

  std::uint32_t update_period_ms = 10;

  OdomSensors() = default;

  /** @brief Start a fluent sensor list with an IMU. */
  explicit OdomSensors(pros::Imu* imu_sensor);

  OdomSensors& with_imu(pros::Imu* imu_sensor, bool use = true);
  OdomSensors& vertical(Wheel* wheel);
  OdomSensors& vertical2(Wheel* wheel);
  OdomSensors& horizontal(Wheel* wheel);
  OdomSensors& horizontal2(Wheel* wheel);

  /**
   * @brief Use averaged drive motor encoders when no tracking wheels exist.
   * @param track_width_in Center-to-center distance between drive sides.
   */
  OdomSensors& with_drive(pros::MotorGroup* left, pros::MotorGroup* right,
                          double track_width_in,
                          double wheel_diameter_in = 3.25,
                          double gear_ratio = 1.0, bool reversed = false);

  OdomSensors& with_update_period(std::uint32_t period_ms);
};

/**
 * @brief Build an @ref OdomSensors bundle from config tables + live wheels.
 *
 * Skips disabled trackers (@c TrackerConfig::port == 0).
 */
OdomSensors make_odom_sensors(pros::Imu* imu, const Wheel* vertical,
                              const Wheel* horizontal = nullptr,
                              const Wheel* vertical2 = nullptr,
                              const Wheel* horizontal2 = nullptr);

/**
 * @struct TrackingWheel
 * @brief A DistanceSensor paired with its mounting offset.
 *
 * For vertical (drive-parallel) wheels @c offset_in is the side-to-side
 * distance from the robot's tracking center. Right is positive.
 *
 * For horizontal (perpendicular) wheels @c offset_in is the front-to-back
 * distance from the tracking center. Forward is positive.
 */
struct TrackingWheel {
  DistanceSensor* sensor = nullptr;  ///< Source of inches traveled.
  double offset_in = 0.0;            ///< Signed offset in inches.

  /** @brief Empty tracking wheel (sensor == nullptr disables it). */
  TrackingWheel() = default;

  /**
   * @param isensor     DistanceSensor implementation.
   * @param ioffset_in  Signed mounting offset in inches.
   */
  TrackingWheel(DistanceSensor* isensor, double ioffset_in)
      : sensor(isensor), offset_in(ioffset_in) {}
};

/**
 * @struct OdomConfig
 * @brief Bag of inputs for Odometry: tracking wheels, drive encoders, IMU.
 *
 * Fields default to "disabled" (nullptr / 0). You only need to set the
 * sensors you actually have. Three setup styles are supported:
 *
 *  1. Direct field assignment (verbose, explicit):
 *     @code
 *     Vortex::OdomConfig cfg;
 *     cfg.vertical_1 = {&vertical_distance, 0.0};
 *     cfg.imu = &imu;
 *     Vortex::Odometry odom(cfg);
 *     @endcode
 *
 *  2. Convenience constructor (best for simple setups):
 *     @code
 *     Vortex::Odometry odom(&imu, &vertical_distance, 0.0);
 *     @endcode
 *
 *  3. Fluent builder (best when mixing features):
 *     @code
 *     Vortex::Odometry odom(Vortex::OdomConfig()
 *                               .with_imu(&imu)
 *                               .with_vertical(&v, 0.0)
 *                               .with_horizontal(&h, -4.75)
 *                               .with_update_period(10));
 *     @endcode
 */
struct OdomConfig {
  TrackingWheel vertical_1;    ///< Primary drive-parallel tracking wheel.
  TrackingWheel vertical_2;    ///< Optional second vertical wheel (for IMU-less heading).
  TrackingWheel horizontal_1;  ///< Primary perpendicular tracking wheel.
  TrackingWheel horizontal_2;  ///< Optional second horizontal wheel.

  DistanceSensor* left_drive = nullptr;   ///< Fallback left drive encoder.
  DistanceSensor* right_drive = nullptr;  ///< Fallback right drive encoder.
  double drive_track_width_in = 0.0;      ///< Distance between drive sides, in inches.

  pros::Imu* imu = nullptr;                ///< Optional inertial sensor.
  bool use_imu = true;                     ///< Toggle IMU heading fusion.
  std::uint32_t update_period_ms = 10;     ///< Loop period in milliseconds.

  /** @brief Construct an empty config (everything disabled). */
  OdomConfig() = default;

  /**
   * @brief Convenience constructor: IMU + one vertical tracking wheel.
   * @param imu_sensor           Inertial sensor for heading.
   * @param vertical_sensor      Drive-parallel tracking wheel sensor.
   * @param vertical_offset_in   Side-to-side offset from tracking center.
   */
  OdomConfig(pros::Imu* imu_sensor, DistanceSensor* vertical_sensor,
             double vertical_offset_in);

  /**
   * @brief Convenience constructor: IMU + one vertical + one horizontal wheel.
   * @param imu_sensor             Inertial sensor for heading.
   * @param vertical_sensor        Drive-parallel tracking wheel sensor.
   * @param vertical_offset_in     Side-to-side offset from tracking center.
   * @param horizontal_sensor      Perpendicular tracking wheel sensor.
   * @param horizontal_offset_in   Front/back offset from tracking center.
   */
  OdomConfig(pros::Imu* imu_sensor, DistanceSensor* vertical_sensor,
             double vertical_offset_in, DistanceSensor* horizontal_sensor,
             double horizontal_offset_in);

  /**
   * @brief Set or replace the IMU.
   * @param imu_sensor IMU to use, or nullptr to disable.
   * @param use        Whether the IMU should drive heading fusion.
   * @return Reference to this config for chaining.
   */
  OdomConfig& with_imu(pros::Imu* imu_sensor, bool use = true);

  /**
   * @brief Set the primary vertical tracking wheel.
   * @return Reference to this config for chaining.
   */
  OdomConfig& with_vertical(DistanceSensor* sensor, double offset_in);

  /**
   * @brief Set the secondary vertical wheel (used to derive heading without IMU).
   * @return Reference to this config for chaining.
   */
  OdomConfig& with_second_vertical(DistanceSensor* sensor, double offset_in);

  /**
   * @brief Set the primary horizontal (perpendicular) tracking wheel.
   * @return Reference to this config for chaining.
   */
  OdomConfig& with_horizontal(DistanceSensor* sensor, double offset_in);

  /**
   * @brief Set the secondary horizontal wheel.
   * @return Reference to this config for chaining.
   */
  OdomConfig& with_second_horizontal(DistanceSensor* sensor, double offset_in);

  /**
   * @brief Use drive motor encoders as a fallback when no tracking wheels exist.
   * @param left_sensor    Distance source for the left drive side.
   * @param right_sensor   Distance source for the right drive side.
   * @param track_width_in Center-to-center distance between drive sides.
   * @return Reference to this config for chaining.
   */
  OdomConfig& with_drive_encoders(DistanceSensor* left_sensor,
                                  DistanceSensor* right_sensor,
                                  double track_width_in);

  /**
   * @brief Override the update task period.
   * @param period_ms Milliseconds between fusion ticks.
   * @return Reference to this config for chaining.
   */
  OdomConfig& with_update_period(std::uint32_t period_ms);
};

/**
 * @class Odometry
 * @brief Pose estimator that fuses tracking wheels, drive encoders, and an IMU.
 *
 * Typical lifecycle (simple):
 *   1. Construct with @ref OdomSensors (see @ref Wheel).
 *   2. Call init() once in @c initialize() — calibrates, zeros pose, starts task.
 *   3. Call reset() at the start of each autonomous routine.
 *   4. Read get_pose() (or x(), y(), heading_deg()) any time.
 *
 * Advanced users can still use @ref OdomConfig and call calibrate(),
 * set_pose(), and start_task() separately.
 */
class Odometry {
 public:
  /** @brief Construct an Odometry with no sensors configured. */
  Odometry();

  /**
   * @brief LemLib-style setup from a sensor bundle.
   * @param sensors Tracking wheels, optional drive encoders, and IMU.
   */
  explicit Odometry(const OdomSensors& sensors);

  /**
   * @brief Construct from a fully-populated config.
   * @param config Sensor + behaviour bundle.
   */
  explicit Odometry(const OdomConfig& config);

  /**
   * @brief Quick setup: IMU + one vertical tracking wheel.
   * @param imu_sensor           Inertial sensor for heading.
   * @param vertical_sensor      Drive-parallel tracking wheel sensor.
   * @param vertical_offset_in   Side-to-side offset from tracking center.
   */
  Odometry(pros::Imu* imu_sensor, DistanceSensor* vertical_sensor,
           double vertical_offset_in);

  /**
   * @brief Quick setup: IMU + one vertical + one horizontal tracking wheel.
   * @param imu_sensor             Inertial sensor for heading.
   * @param vertical_sensor        Drive-parallel tracking wheel sensor.
   * @param vertical_offset_in     Side-to-side offset from tracking center.
   * @param horizontal_sensor      Perpendicular tracking wheel sensor.
   * @param horizontal_offset_in   Front/back offset from tracking center.
   */
  Odometry(pros::Imu* imu_sensor, DistanceSensor* vertical_sensor,
           double vertical_offset_in, DistanceSensor* horizontal_sensor,
           double horizontal_offset_in);

  /**
   * @brief Replace the entire configuration at runtime.
   *
   * Useful if a sensor swap or recalibration requires changing offsets
   * mid-program. Sensor baselines are re-captured automatically.
   */
  void configure(const OdomConfig& config);

  /** @brief Read back the current configuration. */
  OdomConfig get_config() const;

  /**
   * @brief One-call startup: calibrate, set pose, and start the update task.
   *
   * Equivalent to calibrate() + set_pose() + start_task(). This is the
   * recommended entry point in @c initialize(), similar to EZ-Template's
   * @c odom_reset() flow combined with task startup.
   *
   * @param x_in          Starting x in inches.
   * @param y_in          Starting y in inches.
   * @param heading_deg   Starting heading in degrees.
   * @param reset_imu     Reset and wait for the IMU when one is configured.
   * @param start_background_task  Spawn the background fusion task.
   * @param task_name              PROS task name when the task is started.
   */
  void init(double x_in = 0.0, double y_in = 0.0, double heading_deg = 0.0,
            bool reset_imu = true, bool start_background_task = true,
            const char* task_name = "Vortex");

  /**
   * @brief Reset the field pose (EZ-Template-style @c odom_reset).
   * @param x_in        x in inches.
   * @param y_in        y in inches.
   * @param heading_deg Heading in degrees.
   */
  void reset(double x_in = 0.0, double y_in = 0.0, double heading_deg = 0.0);

  /**
   * @brief Calibrate the odometry. Call once in @c initialize().
   * @param reset_imu If true, reset the IMU and wait for it to settle.
   */
  void calibrate(bool reset_imu = true);

  /** @brief Zero every distance sensor and re-cache their baselines. */
  void reset_sensors();

  /** @brief Set the current pose to @p pose (theta in radians). */
  void set_pose(const Pose& pose);

  /**
   * @brief Set position + heading using degrees (most common overload).
   * @param x_in        x in inches.
   * @param y_in        y in inches.
   * @param heading_deg Heading in degrees, clockwise from +Y.
   */
  void set_pose(double x_in, double y_in, double heading_deg);

  /**
   * @brief Translate the robot without changing its heading.
   * @param x_in x in inches.
   * @param y_in y in inches.
   */
  void set_pose(double x_in, double y_in);

  /**
   * @brief Set position + heading using radians.
   * @param x_in     x in inches.
   * @param y_in     y in inches.
   * @param theta_rad Heading in radians.
   */
  void set_pose_rad(double x_in, double y_in, double theta_rad);

  /** @brief Set only the x coordinate (inches). */
  void set_x(double x_in);

  /** @brief Set only the y coordinate (inches). */
  void set_y(double y_in);

  /** @brief Set only the heading (degrees). */
  void set_heading_deg(double heading_deg);

  /** @brief Set only the heading (radians). */
  void set_heading_rad(double theta_rad);

  /** @brief Current pose snapshot. */
  Pose get_pose() const;

  /** @brief Current x position in inches. */
  double x() const;

  /** @brief Current y position in inches. */
  double y() const;

  /** @brief Current heading in radians. */
  double theta_rad() const;

  /** @brief Current heading in degrees. */
  double heading_deg() const;

  /**
   * @brief Run one fusion tick.
   *
   * Normally called automatically by the task started in start_task().
   * Exposed for unit testing and for users who want their own scheduler.
   */
  void update();

  /**
   * @brief Print the current pose to the PROS terminal.
   *
   * Formats the pose beautifully and prints it. Useful for debugging in opcontrol.
   */
  void print() const;

  /**
   * @brief Spawn a background PROS task that calls update() forever.
   * @param task_name Display name used by PROS (default "Vortex").
   */
  void start_task(const char* task_name = "Vortex");

  /** @brief Stop and tear down the background task. */
  void stop_task();

 private:
  static void task_entry(void* params);

  double read_tracking(const TrackingWheel& wheel) const;
  void capture_sensor_positions();
  bool has_vertical_1() const;
  bool has_vertical_2() const;
  bool has_horizontal_1() const;
  bool has_horizontal_2() const;
  bool has_drive_pair() const;

  static OdomConfig config_from_sensors(const OdomSensors& sensors,
                                        std::unique_ptr<MotorGroupDistanceSensor>& left,
                                        std::unique_ptr<MotorGroupDistanceSensor>& right);

  OdomConfig config_;
  Pose pose_;

  std::unique_ptr<MotorGroupDistanceSensor> owned_left_drive_;
  std::unique_ptr<MotorGroupDistanceSensor> owned_right_drive_;

  double last_vertical_1_ = 0.0;
  double last_vertical_2_ = 0.0;
  double last_horizontal_1_ = 0.0;
  double last_horizontal_2_ = 0.0;
  double last_left_drive_ = 0.0;
  double last_right_drive_ = 0.0;
  double last_imu_rotation_rad_ = 0.0;

  bool running_ = false;
  pros::Task* task_ = nullptr;
};

}  // namespace Vortex
