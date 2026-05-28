#include "odom.hpp"

#include <cmath>
#include <vector>

namespace Vortex {

double deg_to_rad(double deg) { return deg * kPi / 180.0; }

double rad_to_deg(double rad) { return rad * 180.0 / kPi; }

double wrap_deg(double deg) {
  while (deg > 180.0) deg -= 360.0;
  while (deg <= -180.0) deg += 360.0;
  return deg;
}

double wrap_rad(double rad) { return deg_to_rad(wrap_deg(rad_to_deg(rad))); }

double angle_error_deg(double target_deg, double current_deg) {
  return wrap_deg(target_deg - current_deg);
}

double distance_between(double x1, double y1, double x2, double y2) {
  const double dx = x2 - x1;
  const double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

double bearing_to_point_deg(double from_x, double from_y, double to_x,
                            double to_y) {
  return rad_to_deg(std::atan2(to_x - from_x, to_y - from_y));
}

double Pose::heading_deg() const { return rad_to_deg(theta); }

void Pose::set_heading_deg(double heading_deg) {
  theta = deg_to_rad(heading_deg);
}

Pose Pose::from_degrees(double x_in, double y_in, double heading_deg) {
  return Pose(x_in, y_in, deg_to_rad(heading_deg));
}

RotationDistanceSensor::RotationDistanceSensor(pros::Rotation* sensor,
                                               double wheel_diameter_in,
                                               double gear_ratio, bool reversed)
    : sensor_(sensor),
      wheel_diameter_in_(wheel_diameter_in),
      gear_ratio_(gear_ratio == 0.0 ? 1.0 : gear_ratio),
      reversed_(reversed),
      offset_degrees_(0.0) {}

double RotationDistanceSensor::get_distance_in() const {
  if (sensor_ == nullptr) return 0.0;

  const double raw_degrees = static_cast<double>(sensor_->get_position()) / 100.0;
  const double degrees = raw_degrees - offset_degrees_;
  const double wheel_revolutions = degrees / 360.0 / gear_ratio_;
  return wheel_revolutions * wheel_diameter_in_ * kPi;
}

void RotationDistanceSensor::reset() {
  if (sensor_ == nullptr) return;
  sensor_->set_reversed(reversed_);
  sensor_->set_data_rate(5);
  offset_degrees_ = static_cast<double>(sensor_->get_position()) / 100.0;
}

MotorGroupDistanceSensor::MotorGroupDistanceSensor(
    pros::MotorGroup* motors, double wheel_diameter_in,
    double wheel_rev_per_motor_rev, bool reversed)
    : motors_(motors),
      wheel_diameter_in_(wheel_diameter_in),
      wheel_rev_per_motor_rev_(wheel_rev_per_motor_rev == 0.0
                                   ? 1.0
                                   : wheel_rev_per_motor_rev),
      direction_(reversed ? -1 : 1),
      offset_degrees_(0.0) {}

double MotorGroupDistanceSensor::get_distance_in() const {
  if (motors_ == nullptr) return 0.0;

#if __cplusplus >= 202002L
  const std::vector<double> positions = motors_->get_position_all();
#else
  const std::vector<double> positions = motors_->get_positions();
#endif
  if (positions.empty()) return 0.0;

  double sum_degrees = 0.0;
  for (double position : positions) {
    sum_degrees += position;
  }

  const double average_raw_degrees = sum_degrees / positions.size();
  const double average_degrees = average_raw_degrees - offset_degrees_;
  const double wheel_revolutions =
      average_degrees / 360.0 * wheel_rev_per_motor_rev_;
  return direction_ * wheel_revolutions * wheel_diameter_in_ * kPi;
}

void MotorGroupDistanceSensor::reset() {
  if (motors_ == nullptr) return;
#if __cplusplus >= 202002L
  const std::vector<double> positions = motors_->get_position_all();
#else
  const std::vector<double> positions = motors_->get_positions();
#endif
  if (positions.empty()) {
    offset_degrees_ = 0.0;
    return;
  }

  double sum_degrees = 0.0;
  for (double position : positions) {
    sum_degrees += position;
  }
  offset_degrees_ = sum_degrees / positions.size();
}

Wheel::Wheel(int rotation_port, double wheel_diameter_in, double offset_in,
             double gear_ratio, bool reversed)
    : offset_in_(offset_in),
      rotation_(std::make_unique<pros::Rotation>(rotation_port)),
      rotation_distance_(std::make_unique<RotationDistanceSensor>(
          rotation_.get(), wheel_diameter_in, gear_ratio, reversed)),
      sensor_(rotation_distance_.get()) {}

Wheel::Wheel(pros::MotorGroup* motors, double wheel_diameter_in, double offset_in,
             double gear_ratio, bool reversed)
    : offset_in_(offset_in),
      motor_distance_(std::make_unique<MotorGroupDistanceSensor>(
          motors, wheel_diameter_in, gear_ratio, reversed)),
      sensor_(motor_distance_.get()) {}

Wheel::Wheel(const TrackerConfig& config) : offset_in_(config.offset_in) {
  if (!config.enabled()) return;

  rotation_ = std::make_unique<pros::Rotation>(config.port);
  rotation_distance_ = std::make_unique<RotationDistanceSensor>(
      rotation_.get(), config.diameter_in, config.gear_ratio, config.reversed);
  sensor_ = rotation_distance_.get();
}

DistanceSensor* Wheel::sensor() const { return sensor_; }

double Wheel::offset_in() const { return offset_in_; }

pros::Rotation* Wheel::rotation() const { return rotation_.get(); }

OdomSensors::OdomSensors(pros::Imu* imu_sensor) : imu(imu_sensor) {}

OdomSensors& OdomSensors::with_imu(pros::Imu* imu_sensor, bool use) {
  imu = imu_sensor;
  use_imu = use;
  return *this;
}

OdomSensors& OdomSensors::vertical(Wheel* wheel) {
  vertical = wheel;
  return *this;
}

OdomSensors& OdomSensors::vertical2(Wheel* wheel) {
  vertical2 = wheel;
  return *this;
}

OdomSensors& OdomSensors::horizontal(Wheel* wheel) {
  horizontal = wheel;
  return *this;
}

OdomSensors& OdomSensors::horizontal2(Wheel* wheel) {
  horizontal2 = wheel;
  return *this;
}

OdomSensors& OdomSensors::with_drive(pros::MotorGroup* left, pros::MotorGroup* right,
                                     double track_width_in, double wheel_diameter_in,
                                     double gear_ratio, bool reversed) {
  left_drive = left;
  right_drive = right;
  drive_track_width_in = track_width_in;
  drive_wheel_diameter_in = wheel_diameter_in;
  drive_gear_ratio = gear_ratio;
  drive_reversed = reversed;
  return *this;
}

OdomSensors& OdomSensors::with_update_period(std::uint32_t period_ms) {
  update_period_ms = period_ms;
  return *this;
}

OdomSensors make_odom_sensors(pros::Imu* imu, const Wheel* vertical,
                              const Wheel* horizontal, const Wheel* vertical2,
                              const Wheel* horizontal2) {
  OdomSensors sensors(imu);

  auto attach = [&](const Wheel* wheel, auto setter) {
    if (wheel != nullptr && wheel->sensor() != nullptr) {
      setter(const_cast<Wheel*>(wheel));
    }
  };

  attach(vertical, [&](Wheel* wheel) { sensors.vertical(wheel); });
  attach(horizontal, [&](Wheel* wheel) { sensors.horizontal(wheel); });
  attach(vertical2, [&](Wheel* wheel) { sensors.vertical2(wheel); });
  attach(horizontal2, [&](Wheel* wheel) { sensors.horizontal2(wheel); });

  return sensors;
}

OdomConfig Odometry::config_from_sensors(
    const OdomSensors& sensors,
    std::unique_ptr<MotorGroupDistanceSensor>& owned_left,
    std::unique_ptr<MotorGroupDistanceSensor>& owned_right) {
  OdomConfig config;
  config.update_period_ms = sensors.update_period_ms;

  if (sensors.imu != nullptr) {
    config.with_imu(sensors.imu, sensors.use_imu);
  }
  if (sensors.vertical != nullptr && sensors.vertical->sensor() != nullptr) {
    config.with_vertical(sensors.vertical->sensor(), sensors.vertical->offset_in());
  }
  if (sensors.vertical2 != nullptr && sensors.vertical2->sensor() != nullptr) {
    config.with_second_vertical(sensors.vertical2->sensor(),
                                sensors.vertical2->offset_in());
  }
  if (sensors.horizontal != nullptr && sensors.horizontal->sensor() != nullptr) {
    config.with_horizontal(sensors.horizontal->sensor(),
                           sensors.horizontal->offset_in());
  }
  if (sensors.horizontal2 != nullptr && sensors.horizontal2->sensor() != nullptr) {
    config.with_second_horizontal(sensors.horizontal2->sensor(),
                                  sensors.horizontal2->offset_in());
  }
  if (sensors.left_drive != nullptr && sensors.right_drive != nullptr &&
      sensors.drive_track_width_in > 0.0) {
    owned_left = std::make_unique<MotorGroupDistanceSensor>(
        sensors.left_drive, sensors.drive_wheel_diameter_in,
        sensors.drive_gear_ratio, sensors.drive_reversed);
    owned_right = std::make_unique<MotorGroupDistanceSensor>(
        sensors.right_drive, sensors.drive_wheel_diameter_in,
        sensors.drive_gear_ratio, sensors.drive_reversed);
    config.with_drive_encoders(owned_left.get(), owned_right.get(),
                               sensors.drive_track_width_in);
  }

  return config;
}

OdomConfig::OdomConfig(pros::Imu* imu_sensor, DistanceSensor* vertical_sensor,
                       double vertical_offset_in)
    : vertical_1(vertical_sensor, vertical_offset_in), imu(imu_sensor) {}

OdomConfig::OdomConfig(pros::Imu* imu_sensor, DistanceSensor* vertical_sensor,
                       double vertical_offset_in,
                       DistanceSensor* horizontal_sensor,
                       double horizontal_offset_in)
    : vertical_1(vertical_sensor, vertical_offset_in),
      horizontal_1(horizontal_sensor, horizontal_offset_in),
      imu(imu_sensor) {}

OdomConfig& OdomConfig::with_imu(pros::Imu* imu_sensor, bool use) {
  imu = imu_sensor;
  use_imu = use;
  return *this;
}

OdomConfig& OdomConfig::with_vertical(DistanceSensor* sensor, double offset_in) {
  vertical_1 = TrackingWheel(sensor, offset_in);
  return *this;
}

OdomConfig& OdomConfig::with_second_vertical(DistanceSensor* sensor,
                                             double offset_in) {
  vertical_2 = TrackingWheel(sensor, offset_in);
  return *this;
}

OdomConfig& OdomConfig::with_horizontal(DistanceSensor* sensor,
                                        double offset_in) {
  horizontal_1 = TrackingWheel(sensor, offset_in);
  return *this;
}

OdomConfig& OdomConfig::with_second_horizontal(DistanceSensor* sensor,
                                               double offset_in) {
  horizontal_2 = TrackingWheel(sensor, offset_in);
  return *this;
}

OdomConfig& OdomConfig::with_drive_encoders(DistanceSensor* left_sensor,
                                            DistanceSensor* right_sensor,
                                            double track_width_in) {
  left_drive = left_sensor;
  right_drive = right_sensor;
  drive_track_width_in = track_width_in;
  return *this;
}

OdomConfig& OdomConfig::with_update_period(std::uint32_t period_ms) {
  update_period_ms = period_ms;
  return *this;
}

Odometry::Odometry() = default;

Odometry::Odometry(const OdomSensors& sensors)
    : config_(config_from_sensors(sensors, owned_left_drive_, owned_right_drive_)) {}

Odometry::Odometry(const OdomConfig& config) : config_(config) {}

Odometry::Odometry(pros::Imu* imu_sensor, DistanceSensor* vertical_sensor,
                   double vertical_offset_in)
    : config_(imu_sensor, vertical_sensor, vertical_offset_in) {}

Odometry::Odometry(pros::Imu* imu_sensor, DistanceSensor* vertical_sensor,
                   double vertical_offset_in,
                   DistanceSensor* horizontal_sensor,
                   double horizontal_offset_in)
    : config_(imu_sensor, vertical_sensor, vertical_offset_in,
              horizontal_sensor, horizontal_offset_in) {}

void Odometry::configure(const OdomConfig& config) {
  config_ = config;
  capture_sensor_positions();
}

OdomConfig Odometry::get_config() const { return config_; }

void Odometry::init(double x_in, double y_in, double heading_deg, bool reset_imu,
                    bool start_background_task, const char* task_name) {
  calibrate(reset_imu);
  set_pose(x_in, y_in, heading_deg);
  if (start_background_task) {
    start_task(task_name);
  }
}

void Odometry::reset(double x_in, double y_in, double heading_deg) {
  set_pose(x_in, y_in, heading_deg);
}

void Odometry::calibrate(bool reset_imu) {
  if (config_.imu != nullptr && config_.use_imu && reset_imu) {
    config_.imu->reset();
    pros::delay(200); // Give the IMU time to initiate calibration and avoid race conditions
    while (config_.imu->is_calibrating()) {
      pros::delay(10);
    }
  }

  reset_sensors();
}

void Odometry::reset_sensors() {
  if (has_vertical_1()) config_.vertical_1.sensor->reset();
  if (has_vertical_2()) config_.vertical_2.sensor->reset();
  if (has_horizontal_1()) config_.horizontal_1.sensor->reset();
  if (has_horizontal_2()) config_.horizontal_2.sensor->reset();
  if (config_.left_drive != nullptr) config_.left_drive->reset();
  if (config_.right_drive != nullptr) config_.right_drive->reset();

  capture_sensor_positions();
}

void Odometry::set_pose(const Pose& pose) {
  pose_ = pose;
  capture_sensor_positions();
}

void Odometry::set_pose(double x_in, double y_in, double heading_deg) {
  set_pose(Pose::from_degrees(x_in, y_in, heading_deg));
}

void Odometry::set_pose(double x_in, double y_in) {
  set_pose(Pose(x_in, y_in, pose_.theta));
}

void Odometry::set_pose_rad(double x_in, double y_in, double theta_rad) {
  set_pose(Pose(x_in, y_in, theta_rad));
}

void Odometry::set_x(double x_in) { set_pose(Pose(x_in, pose_.y, pose_.theta)); }

void Odometry::set_y(double y_in) { set_pose(Pose(pose_.x, y_in, pose_.theta)); }

void Odometry::set_heading_deg(double heading_deg) {
  set_pose(Pose(pose_.x, pose_.y, deg_to_rad(heading_deg)));
}

void Odometry::set_heading_rad(double theta_rad) {
  set_pose(Pose(pose_.x, pose_.y, theta_rad));
}

Pose Odometry::get_pose() const { return pose_; }

double Odometry::x() const { return pose_.x; }

double Odometry::y() const { return pose_.y; }

double Odometry::theta_rad() const { return pose_.theta; }

double Odometry::heading_deg() const { return pose_.heading_deg(); }

void Odometry::update() {
  const double vertical_1 = read_tracking(config_.vertical_1);
  const double vertical_2 = read_tracking(config_.vertical_2);
  const double horizontal_1 = read_tracking(config_.horizontal_1);
  const double horizontal_2 = read_tracking(config_.horizontal_2);
  const double left_drive =
      config_.left_drive == nullptr ? 0.0 : config_.left_drive->get_distance_in();
  const double right_drive = config_.right_drive == nullptr
                                 ? 0.0
                                 : config_.right_drive->get_distance_in();

  const double d_vertical_1 = vertical_1 - last_vertical_1_;
  const double d_vertical_2 = vertical_2 - last_vertical_2_;
  const double d_horizontal_1 = horizontal_1 - last_horizontal_1_;
  const double d_horizontal_2 = horizontal_2 - last_horizontal_2_;
  const double d_left = left_drive - last_left_drive_;
  const double d_right = right_drive - last_right_drive_;

  const double previous_heading = pose_.theta;
  double dtheta = 0.0;

  if (config_.imu != nullptr && config_.use_imu) {
    const double current_imu_rotation = deg_to_rad(config_.imu->get_rotation());
    dtheta = current_imu_rotation - last_imu_rotation_rad_;
    last_imu_rotation_rad_ = current_imu_rotation;
  } else if (has_vertical_1() && has_vertical_2()) {
    const double denominator =
        config_.vertical_2.offset_in - config_.vertical_1.offset_in;
    if (std::fabs(denominator) > 0.0001) {
      dtheta = (d_vertical_1 - d_vertical_2) / denominator;
    }
  } else if (has_horizontal_1() && has_horizontal_2()) {
    const double denominator =
        config_.horizontal_1.offset_in - config_.horizontal_2.offset_in;
    if (std::fabs(denominator) > 0.0001) {
      dtheta = (d_horizontal_1 - d_horizontal_2) / denominator;
    }
  } else if (has_drive_pair()) {
    dtheta = (d_left - d_right) / config_.drive_track_width_in;
  }

  double local_forward = 0.0;
  int forward_sources = 0;

  if (has_vertical_1()) {
    local_forward += d_vertical_1 + config_.vertical_1.offset_in * dtheta;
    forward_sources++;
  }
  if (has_vertical_2()) {
    local_forward += d_vertical_2 + config_.vertical_2.offset_in * dtheta;
    forward_sources++;
  }
  if (forward_sources == 0 && has_drive_pair()) {
    local_forward = (d_left + d_right) / 2.0;
  } else if (forward_sources > 0) {
    local_forward /= forward_sources;
  }

  double local_strafe = 0.0;
  int strafe_sources = 0;

  if (has_horizontal_1()) {
    local_strafe += d_horizontal_1 - config_.horizontal_1.offset_in * dtheta;
    strafe_sources++;
  }
  if (has_horizontal_2()) {
    local_strafe += d_horizontal_2 - config_.horizontal_2.offset_in * dtheta;
    strafe_sources++;
  }
  if (strafe_sources > 0) {
    local_strafe /= strafe_sources;
  }

  const double mid_heading = previous_heading + dtheta / 2.0;
  pose_.x += local_strafe * std::cos(mid_heading) +
             local_forward * std::sin(mid_heading);
  pose_.y += local_forward * std::cos(mid_heading) -
             local_strafe * std::sin(mid_heading);
  pose_.theta = previous_heading + dtheta;

  last_vertical_1_ = vertical_1;
  last_vertical_2_ = vertical_2;
  last_horizontal_1_ = horizontal_1;
  last_horizontal_2_ = horizontal_2;
  last_left_drive_ = left_drive;
  last_right_drive_ = right_drive;
}

void Odometry::print() const {
  const Pose p = get_pose();
  printf("Odometry Pose -> X: %6.2f in, Y: %6.2f in, Heading: %6.1f deg\n",
         p.x, p.y, p.heading_deg());
}

void Odometry::start_task(const char* task_name) {
  if (task_ != nullptr) return;

  running_ = true;
  task_ = new pros::Task(task_entry, this, TASK_PRIORITY_DEFAULT,
                         TASK_STACK_DEPTH_DEFAULT, task_name);
}

void Odometry::stop_task() {
  running_ = false;
  if (task_ == nullptr) return;

  task_->remove();
  delete task_;
  task_ = nullptr;
}

void Odometry::task_entry(void* params) {
  Odometry* odom = static_cast<Odometry*>(params);
  while (odom != nullptr && odom->running_) {
    odom->update();
    pros::delay(odom->config_.update_period_ms);
  }
}

double Odometry::read_tracking(const TrackingWheel& wheel) const {
  return wheel.sensor == nullptr ? 0.0 : wheel.sensor->get_distance_in();
}

void Odometry::capture_sensor_positions() {
  last_vertical_1_ = read_tracking(config_.vertical_1);
  last_vertical_2_ = read_tracking(config_.vertical_2);
  last_horizontal_1_ = read_tracking(config_.horizontal_1);
  last_horizontal_2_ = read_tracking(config_.horizontal_2);
  last_left_drive_ =
      config_.left_drive == nullptr ? 0.0 : config_.left_drive->get_distance_in();
  last_right_drive_ =
      config_.right_drive == nullptr ? 0.0 : config_.right_drive->get_distance_in();

  if (config_.imu != nullptr && config_.use_imu) {
    last_imu_rotation_rad_ = deg_to_rad(config_.imu->get_rotation());
  }
}

bool Odometry::has_vertical_1() const {
  return config_.vertical_1.sensor != nullptr;
}

bool Odometry::has_vertical_2() const {
  return config_.vertical_2.sensor != nullptr;
}

bool Odometry::has_horizontal_1() const {
  return config_.horizontal_1.sensor != nullptr;
}

bool Odometry::has_horizontal_2() const {
  return config_.horizontal_2.sensor != nullptr;
}

bool Odometry::has_drive_pair() const {
  return config_.left_drive != nullptr && config_.right_drive != nullptr &&
         config_.drive_track_width_in > 0.0;
}

}  // namespace Vortex
