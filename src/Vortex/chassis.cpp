#include "chassis.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace Vortex {

namespace {
int clamp_int(int value, int low, int high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

double clamp_double(double value, double low, double high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

int sign_of(double value) {
  if (value > 0.0) return 1;
  if (value < 0.0) return -1;
  return 0;
}

std::string motor_tag(char side, std::size_t index) {
  char buffer[8];
  std::snprintf(buffer, sizeof(buffer), "%c%d", side,
                static_cast<int>(index + 1));
  return buffer;
}
}  // namespace

PID::PID(const PIDGains& gains) : gains_(gains) {}

void PID::set_gains(const PIDGains& gains) {
  gains_ = gains;
  reset();
}

PIDGains PID::get_gains() const { return gains_; }

void PID::reset() {
  integral_ = 0.0;
  last_error_ = 0.0;
  first_step_ = true;
}

double PID::step(double error) {
  if (first_step_) {
    last_error_ = error;
    first_step_ = false;
  }

  if (gains_.integral_zone <= 0.0 ||
      std::fabs(error) <= gains_.integral_zone) {
    integral_ += error;
  } else {
    integral_ = 0.0;
  }

  if (gains_.max_integral > 0.0) {
    integral_ = clamp_double(integral_, -gains_.max_integral,
                             gains_.max_integral);
  }

  const double derivative = error - last_error_;
  last_error_ = error;

  return gains_.kP * error + gains_.kI * integral_ + gains_.kD * derivative;
}

Timer::Timer() { reset(); }

void Timer::reset() { start_ms_ = pros::millis(); }

std::uint32_t Timer::elapsed_ms() const { return pros::millis() - start_ms_; }

bool Timer::done(std::uint32_t duration_ms) const {
  return elapsed_ms() >= duration_ms;
}

SlewLimiter::SlewLimiter(double step) : step_(std::fabs(step)) {}

void SlewLimiter::set_step(double step) { step_ = std::fabs(step); }

void SlewLimiter::reset(double value) { value_ = value; }

double SlewLimiter::step(double target) {
  if (target > value_ + step_) value_ += step_;
  else if (target < value_ - step_) value_ -= step_;
  else value_ = target;
  return value_;
}

void ExitCondition::reset() {
  small_since_ = 0;
  big_since_ = 0;
}

bool ExitCondition::settled(double error_abs) {
  const std::uint32_t now = pros::millis();

  if (error_abs <= small_error) {
    if (small_since_ == 0) small_since_ = now;
  } else {
    small_since_ = 0;
  }

  if (error_abs <= big_error) {
    if (big_since_ == 0) big_since_ = now;
  } else {
    big_since_ = 0;
  }

  const bool small_done =
      small_since_ != 0 && now - small_since_ >= small_time_ms;
  const bool big_done = big_since_ != 0 && now - big_since_ >= big_time_ms;
  return small_done || big_done;
}

int ExpoDriveCurve::apply(int input) const {
  const int clamped = clamp_int(input, -127, 127);
  const int sign = clamped < 0 ? -1 : 1;
  const int mag = std::abs(clamped);
  if (mag <= deadband) return 0;

  const double usable = 127.0 - deadband;
  const double normalized = (mag - deadband) / usable;
  const double curved = std::pow(normalized, std::max(0.1, curve));
  const double output = min_output + (127 - min_output) * curved;
  return sign * clamp_int(static_cast<int>(output + 0.5), 0, 127);
}

Path inject_path(const Path& path, double spacing_in) {
  if (path.size() < 2 || spacing_in <= 0.0) return path;

  Path out;
  out.push_back(path.front());

  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const PathPoint& a = path[i];
    const PathPoint& b = path[i + 1];
    const double dist = distance_between(a.x, a.y, b.x, b.y);
    const int steps = std::max(1, static_cast<int>(dist / spacing_in));

    for (int step = 1; step <= steps; ++step) {
      const double t = static_cast<double>(step) / steps;
      out.push_back(PathPoint(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
                              b.speed));
    }
  }

  return out;
}

Path smooth_path(const Path& path, double weight_data, double weight_smooth,
                 double tolerance) {
  if (path.size() < 3) return path;

  Path smoothed = path;
  double change = tolerance;

  while (change >= tolerance) {
    change = 0.0;
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
      const double old_x = smoothed[i].x;
      const double old_y = smoothed[i].y;

      smoothed[i].x += weight_data * (path[i].x - smoothed[i].x);
      smoothed[i].y += weight_data * (path[i].y - smoothed[i].y);
      smoothed[i].x += weight_smooth *
                       (smoothed[i - 1].x + smoothed[i + 1].x -
                        2.0 * smoothed[i].x);
      smoothed[i].y += weight_smooth *
                       (smoothed[i - 1].y + smoothed[i + 1].y -
                        2.0 * smoothed[i].y);

      change += std::fabs(old_x - smoothed[i].x) +
                std::fabs(old_y - smoothed[i].y);
    }
  }

  return smoothed;
}

Path make_path(std::initializer_list<PathPoint> points) {
  return Path(points.begin(), points.end());
}

Path load_path(const char* filename) {
  Path path;
  std::FILE* file = std::fopen(filename, "r");
  if (file == nullptr) return path;

  char line[96];
  while (std::fgets(line, sizeof(line), file) != nullptr) {
    for (char* cursor = line; *cursor != '\0'; ++cursor) {
      if (*cursor == ',') *cursor = ' ';
    }

    double x = 0.0;
    double y = 0.0;
    int speed = 127;
    const int count = std::sscanf(line, "%lf %lf %d", &x, &y, &speed);
    if (count >= 2) path.push_back(PathPoint(x, y, speed));
  }

  std::fclose(file);
  return path;
}

Chassis::Chassis(pros::MotorGroup* left, pros::MotorGroup* right,
                 Odometry* odom)
    : left_(left),
      right_(right),
      odom_(odom),
      drive_pid_(config_.drive),
      turn_pid_(config_.turn),
      swing_pid_(config_.swing),
      point_drive_pid_(config_.point_drive),
      point_turn_pid_(config_.point_turn),
      pose_drive_pid_(config_.pose_drive),
      pose_turn_pid_(config_.pose_turn),
      follow_drive_pid_(config_.follow_drive),
      follow_turn_pid_(config_.follow_turn) {}

Chassis::Chassis(pros::MotorGroup* left, pros::MotorGroup* right,
                 Odometry* odom, const std::vector<int>& left_ports,
                 const std::vector<int>& right_ports)
    : Chassis(left, right, odom) {
  set_drive_motor_ports(left_ports, right_ports);
}

Chassis::Chassis(pros::MotorGroup* left, pros::MotorGroup* right, Odometry* odom,
                 const DrivetrainConfig& drivetrain)
    : Chassis(left, right, odom, drivetrain.left_ports, drivetrain.right_ports) {}

void Chassis::init(pros::motor_brake_mode_e_t brake_mode) {
  set_brake_mode(brake_mode);
  set_drive_curves(ExpoDriveCurve{}, ExpoDriveCurve{});
}

void Chassis::set_motion_config(const MotionConfig& config) {
  config_ = config;
  drive_pid_.set_gains(config_.drive);
  turn_pid_.set_gains(config_.turn);
  swing_pid_.set_gains(config_.swing);
  point_drive_pid_.set_gains(config_.point_drive);
  point_turn_pid_.set_gains(config_.point_turn);
  pose_drive_pid_.set_gains(config_.pose_drive);
  pose_turn_pid_.set_gains(config_.pose_turn);
  follow_drive_pid_.set_gains(config_.follow_drive);
  follow_turn_pid_.set_gains(config_.follow_turn);
}

MotionConfig Chassis::get_motion_config() const { return config_; }

MotionConfig* Chassis::motion_config() { return &config_; }

void Chassis::set_safety_config(const SafetyConfig& config) {
  safety_config_ = config;
}

SafetyConfig Chassis::get_safety_config() const { return safety_config_; }

const char* Chassis::last_safety_message() const { return safety_message_; }

void Chassis::set_drive_groups(pros::MotorGroup* left,
                               pros::MotorGroup* right) {
  cancelMotion();
  left_ = left;
  right_ = right;
}

void Chassis::set_drive_motor_ports(const std::vector<int>& left_ports,
                                    const std::vector<int>& right_ports) {
  left_drive_ports_ = left_ports;
  right_drive_ports_ = right_ports;
}

void Chassis::set_drive_curves(const ExpoDriveCurve& throttle,
                               const ExpoDriveCurve& turn) {
  throttle_curve_ = throttle;
  turn_curve_ = turn;
}

void Chassis::set_brake_mode(pros::motor_brake_mode_e_t mode) {
#if __cplusplus >= 202002L
  if (left_ != nullptr) left_->set_brake_mode_all(mode);
  if (right_ != nullptr) right_->set_brake_mode_all(mode);
#else
  if (left_ != nullptr) left_->set_brake_modes(mode);
  if (right_ != nullptr) right_->set_brake_modes(mode);
#endif
}

void Chassis::tank_voltage(int left_mv, int right_mv) {
  if (!safety_ok(left_mv, right_mv)) {
    if (left_ != nullptr) left_->move_voltage(0);
    if (right_ != nullptr) right_->move_voltage(0);
    return;
  }

  if (left_ != nullptr) left_->move_voltage(clamp_int(left_mv, -12000, 12000));
  if (right_ != nullptr) right_->move_voltage(clamp_int(right_mv, -12000, 12000));
}

void Chassis::tank(int left, int right, bool disable_drive_curve) {
  const int l = disable_drive_curve ? left : throttle_curve_.apply(left);
  const int r = disable_drive_curve ? right : throttle_curve_.apply(right);
  tank_voltage(l * 12000 / 127, r * 12000 / 127);
}

void Chassis::arcade(int throttle, int turn, bool disable_drive_curve,
                     double desaturate_bias) {
  double t = disable_drive_curve ? clamp_int(throttle, -127, 127)
                                 : throttle_curve_.apply(throttle);
  double r = disable_drive_curve ? clamp_int(turn, -127, 127)
                                 : turn_curve_.apply(turn);

  double left = t + r;
  double right = t - r;
  const double max_mag = std::max(std::fabs(left), std::fabs(right));

  if (max_mag > 127.0) {
    const double bias = clamp_double(desaturate_bias, 0.0, 1.0);
    if (bias <= 0.01) {
      const double allowed_turn = 127.0 - std::fabs(t);
      r = sign_of(r) * std::max(0.0, allowed_turn);
    } else if (bias >= 0.99) {
      const double allowed_throttle = 127.0 - std::fabs(r);
      t = sign_of(t) * std::max(0.0, allowed_throttle);
    } else {
      const double scale = 127.0 / max_mag;
      t *= scale;
      r *= scale;
    }
    left = t + r;
    right = t - r;
  }

  tank_voltage(static_cast<int>(left * 12000.0 / 127.0),
               static_cast<int>(right * 12000.0 / 127.0));
}

void Chassis::curvature(int throttle, int turn, bool disable_drive_curve) {
  const int t = disable_drive_curve ? clamp_int(throttle, -127, 127)
                                    : throttle_curve_.apply(throttle);
  const int r = disable_drive_curve ? clamp_int(turn, -127, 127)
                                    : turn_curve_.apply(turn);

  if (std::abs(t) < 5) {
    arcade(0, r, true);
    return;
  }

  const double angular = std::fabs(t) * r / 127.0;
  arcade(t, static_cast<int>(angular), true);
}

void Chassis::brake() {
  if (left_ != nullptr) left_->brake();
  if (right_ != nullptr) right_->brake();
}

bool Chassis::drive_distance(double distance_in, std::uint32_t timeout_ms,
                             int max_voltage) {
  return run_drive_distance(distance_in, timeout_ms, max_voltage);
}

bool Chassis::turn_to_heading(double heading_deg, std::uint32_t timeout_ms,
                              int max_voltage) {
  TurnToHeadingParams params;
  params.max_speed = voltage_to_speed(max_voltage);
  set_target("Turn", 0.0, 0.0, heading_deg);
  return run_turn_to_heading(heading_deg, timeout_ms, params);
}

bool Chassis::turn_to_point(double x_in, double y_in, std::uint32_t timeout_ms,
                            int max_voltage) {
  if (odom_ == nullptr) return false;
  TurnToPointParams params;
  params.max_speed = voltage_to_speed(max_voltage);
  const Pose pose = odom_->get_pose();
  double target = bearing_to_point_deg(pose.x, pose.y, x_in, y_in);
  if (!params.forwards) target += 180.0;
  set_target("TurnPt", x_in, y_in, target);
  TurnToHeadingParams turn_params;
  turn_params.direction = params.direction;
  turn_params.max_speed = params.max_speed;
  turn_params.min_speed = params.min_speed;
  turn_params.early_exit_range_deg = params.early_exit_range_deg;
  return run_turn_to_heading(target, timeout_ms, turn_params);
}

bool Chassis::drive_to_point(double x_in, double y_in, std::uint32_t timeout_ms,
                             int max_voltage) {
  MoveToPointParams params;
  params.max_speed = voltage_to_speed(max_voltage);
  set_target("Point", x_in, y_in);
  return run_move_to_point(x_in, y_in, timeout_ms, params);
}

bool Chassis::drive_to_pose(double x_in, double y_in, double heading_deg,
                            std::uint32_t timeout_ms, int max_voltage,
                            const MoveToPoseParams& params) {
  MoveToPoseParams tuned = params;
  tuned.max_speed = voltage_to_speed(max_voltage);
  set_target("Pose", x_in, y_in, heading_deg);
  return run_move_to_pose(x_in, y_in, heading_deg, timeout_ms, tuned);
}

bool Chassis::follow_path(const Path& path, std::uint32_t timeout_ms,
                          const FollowParams& params) {
  set_target("Follow");
  return run_follow(path, static_cast<int>(timeout_ms), params);
}

void Chassis::turnToHeading(double theta_deg, int timeout_ms,
                            TurnToHeadingParams params, bool async) {
  MotionCommand command;
  command.type = MotionType::TURN_HEADING;
  command.theta = theta_deg;
  command.timeout_ms = timeout_ms;
  command.turn_heading = params;
  dispatch(command, async);
}

void Chassis::turnToPoint(double x_in, double y_in, int timeout_ms,
                          TurnToPointParams params, bool async) {
  MotionCommand command;
  command.type = MotionType::TURN_POINT;
  command.x = x_in;
  command.y = y_in;
  command.timeout_ms = timeout_ms;
  command.turn_point = params;
  dispatch(command, async);
}

void Chassis::swingToHeading(double theta_deg, DriveSide locked_side,
                             int timeout_ms, SwingToHeadingParams params,
                             bool async) {
  MotionCommand command;
  command.type = MotionType::SWING_HEADING;
  command.theta = theta_deg;
  command.timeout_ms = timeout_ms;
  command.locked_side = locked_side;
  command.swing_heading = params;
  dispatch(command, async);
}

void Chassis::swingToPoint(double x_in, double y_in, DriveSide locked_side,
                           int timeout_ms, SwingToPointParams params,
                           bool async) {
  MotionCommand command;
  command.type = MotionType::SWING_POINT;
  command.x = x_in;
  command.y = y_in;
  command.timeout_ms = timeout_ms;
  command.locked_side = locked_side;
  command.swing_point = params;
  dispatch(command, async);
}

void Chassis::moveToPoint(double x_in, double y_in, int timeout_ms,
                          MoveToPointParams params, bool async) {
  MotionCommand command;
  command.type = MotionType::MOVE_POINT;
  command.x = x_in;
  command.y = y_in;
  command.timeout_ms = timeout_ms;
  command.move_point = params;
  dispatch(command, async);
}

void Chassis::moveToPose(double x_in, double y_in, double theta_deg,
                         int timeout_ms, MoveToPoseParams params, bool async) {
  MotionCommand command;
  command.type = MotionType::MOVE_POSE;
  command.x = x_in;
  command.y = y_in;
  command.theta = theta_deg;
  command.timeout_ms = timeout_ms;
  command.move_pose = params;
  dispatch(command, async);
}

void Chassis::follow(const Path& path, int timeout_ms, FollowParams params,
                     bool async) {
  MotionCommand command;
  command.type = MotionType::FOLLOW;
  command.timeout_ms = timeout_ms;
  command.path = path;
  command.follow_params = params;
  dispatch(command, async);
}

void Chassis::cancelMotion() {
  motion_cancel_ = true;
  while (motion_running_) {
    pros::delay(config_.loop_delay_ms);
  }

  if (motion_task_ != nullptr) {
    motion_task_->remove();
    delete motion_task_;
    motion_task_ = nullptr;
  }

  brake();
  set_target("Idle");
}

void Chassis::waitUntil(double progress) {
  while (motion_running_ && motion_progress_ < progress) {
    pros::delay(config_.loop_delay_ms);
  }
}

void Chassis::waitUntilDone() {
  while (motion_running_) {
    pros::delay(config_.loop_delay_ms);
  }
}

bool Chassis::isMotionRunning() const { return motion_running_; }

bool Chassis::lastMotionSucceeded() const { return motion_success_; }

double Chassis::motionProgress() const { return motion_progress_; }

Chassis::MotionTarget Chassis::currentTarget() const {
  return current_target_;
}

std::vector<MotorTemperature> Chassis::motorTemperatures() const {
  std::vector<MotorTemperature> temps;

  for (std::size_t i = 0; i < left_drive_ports_.size(); ++i) {
    const int port = std::abs(left_drive_ports_[i]);
    pros::Motor motor(port);
    const double celsius = motor.get_temperature();
    const int percent = clamp_int(
        static_cast<int>((celsius - 25.0) * 100.0 / 35.0), 0, 100);
    temps.push_back({motor_tag('L', i), port, celsius, percent});
  }

  for (std::size_t i = 0; i < right_drive_ports_.size(); ++i) {
    const int port = std::abs(right_drive_ports_[i]);
    pros::Motor motor(port);
    const double celsius = motor.get_temperature();
    const int percent = clamp_int(
        static_cast<int>((celsius - 25.0) * 100.0 / 35.0), 0, 100);
    temps.push_back({motor_tag('R', i), port, celsius, percent});
  }

  return temps;
}

void Chassis::task_entry(void* params) {
  Chassis* chassis = static_cast<Chassis*>(params);
  if (chassis == nullptr) return;

  chassis->motion_success_ = chassis->run_command(chassis->command_);
  chassis->motion_running_ = false;
  chassis->brake();
  chassis->set_target("Idle");
}

void Chassis::dispatch(const MotionCommand& command, bool async) {
  if (async) {
    start_async(command);
    return;
  }

  cancelMotion();
  motion_cancel_ = false;
  motion_running_ = true;
  motion_progress_ = 0.0;
  motion_success_ = run_command(command);
  motion_running_ = false;
  brake();
  set_target("Idle");
}

void Chassis::start_async(const MotionCommand& command) {
  cancelMotion();
  command_ = command;
  motion_cancel_ = false;
  motion_success_ = false;
  motion_progress_ = 0.0;
  motion_running_ = true;
  motion_task_ = new pros::Task(task_entry, this, TASK_PRIORITY_DEFAULT,
                                TASK_STACK_DEPTH_DEFAULT, "v5motion");
}

bool Chassis::run_command(const MotionCommand& command) {
  switch (command.type) {
    case MotionType::DRIVE_DISTANCE:
      return run_drive_distance(command.distance, command.timeout_ms, command.max_voltage);
    case MotionType::TURN_HEADING:
      set_target("Turn", 0.0, 0.0, command.theta);
      return run_turn_to_heading(command.theta, command.timeout_ms,
                                 command.turn_heading);
    case MotionType::TURN_POINT: {
      if (odom_ == nullptr) return false;
      const Pose pose = odom_->get_pose();
      double target = bearing_to_point_deg(pose.x, pose.y, command.x, command.y);
      if (!command.turn_point.forwards) target += 180.0;
      set_target("TurnPt", command.x, command.y, target);

      TurnToHeadingParams params;
      params.direction = command.turn_point.direction;
      params.max_speed = command.turn_point.max_speed;
      params.min_speed = command.turn_point.min_speed;
      params.early_exit_range_deg = command.turn_point.early_exit_range_deg;
      return run_turn_to_heading(target, command.timeout_ms, params);
    }
    case MotionType::SWING_HEADING:
      set_target("Swing", 0.0, 0.0, command.theta);
      return run_swing_to_heading(command.theta, command.locked_side,
                                  command.timeout_ms, command.swing_heading);
    case MotionType::SWING_POINT: {
      if (odom_ == nullptr) return false;
      const Pose pose = odom_->get_pose();
      double target = bearing_to_point_deg(pose.x, pose.y, command.x, command.y);
      if (!command.swing_point.forwards) target += 180.0;
      set_target("SwingPt", command.x, command.y, target);

      SwingToHeadingParams params;
      params.direction = command.swing_point.direction;
      params.max_speed = command.swing_point.max_speed;
      params.min_speed = command.swing_point.min_speed;
      params.early_exit_range_deg = command.swing_point.early_exit_range_deg;
      return run_swing_to_heading(target, command.locked_side,
                                  command.timeout_ms, params);
    }
    case MotionType::MOVE_POINT:
      set_target("Point", command.x, command.y);
      return run_move_to_point(command.x, command.y, command.timeout_ms,
                               command.move_point);
    case MotionType::MOVE_POSE:
      set_target("Pose", command.x, command.y, command.theta);
      return run_move_to_pose(command.x, command.y, command.theta,
                              command.timeout_ms, command.move_pose);
    case MotionType::FOLLOW:
      if (!command.path.empty()) {
        const PathPoint& target = command.path.back();
        set_target("Follow", target.x, target.y);
      }
      return run_follow(command.path, command.timeout_ms, command.follow_params);
    case MotionType::NONE:
    default:
      return false;
  }
}

bool Chassis::run_drive_distance(double distance_in, std::uint32_t timeout_ms,
                                 int max_voltage) {
  if (odom_ == nullptr) return false;

  drive_pid_.reset();
  turn_pid_.reset();
  set_target("Drive", 0.0, 0.0, 0.0, distance_in);

  const Pose start_pose = odom_->get_pose();
  const double start_heading_deg = start_pose.heading_deg();
  const double heading_rad = start_pose.theta;
  const double total_distance = std::fabs(distance_in);
  const std::uint32_t start_ms = pros::millis();
  ExitCondition exit;
  exit.small_error = config_.drive_settle_error_in;
  exit.small_time_ms = config_.settle_time_ms;
  exit.big_error = config_.drive_settle_error_in * 2.0;
  exit.big_time_ms = config_.settle_time_ms * 2;

  while (!timed_out(start_ms, timeout_ms) && !motion_cancel_) {
    const Pose current = odom_->get_pose();
    const double dx = current.x - start_pose.x;
    const double dy = current.y - start_pose.y;
    const double traveled = dx * std::sin(heading_rad) + dy * std::cos(heading_rad);
    const double distance_error = distance_in - traveled;
    const double heading_error =
        angle_error_deg(start_heading_deg, current.heading_deg());

    const double progress = clamp_double(std::fabs(traveled), 0.0, total_distance);
    const double remaining = std::fabs(distance_error);
    const int profile_limit = trapezoid_limit(
        progress, remaining, max_voltage, config_.trapezoid.min_speed,
        config_.trapezoid.linear_accel_in, config_.trapezoid.linear_decel_in);
    const int drive = apply_trapezoid(
        drive_pid_.step(distance_error), progress, remaining, max_voltage,
        config_.trapezoid.min_speed, config_.trapezoid.linear_accel_in,
        config_.trapezoid.linear_decel_in);
    const int turn = clamp_voltage(turn_pid_.step(heading_error), max_voltage / 2);
    tank_drive_turn_profiled(drive, turn, profile_limit);
    motion_progress_ = progress;

    if (exit.settled(std::fabs(distance_error))) return true;
    pros::delay(config_.loop_delay_ms);
  }

  return false;
}

bool Chassis::run_turn_to_heading(double heading_deg, int timeout_ms,
                                  const TurnToHeadingParams& params) {
  if (odom_ == nullptr) return false;

  turn_pid_.reset();
  const std::uint32_t start_ms = pros::millis();
  const int max_voltage = speed_to_voltage(params.max_speed);
  const double initial_error =
      std::fabs(directed_error(heading_deg, odom_->heading_deg(),
                               params.direction));
  ExitCondition exit;
  exit.small_error = params.early_exit_range_deg > 0.0
                         ? params.early_exit_range_deg
                         : config_.turn_settle_error_deg;
  exit.small_time_ms = config_.settle_time_ms;
  exit.big_error = exit.small_error * 2.0;
  exit.big_time_ms = config_.settle_time_ms * 2;

  while (!timed_out(start_ms, timeout_ms) && !motion_cancel_) {
    const Pose current = odom_->get_pose();
    const double error =
        directed_error(heading_deg, current.heading_deg(), params.direction);
    const double remaining = std::fabs(error);
    const double progress = clamp_double(initial_error - remaining, 0.0,
                                         initial_error);
    int turn = apply_trapezoid(
        turn_pid_.step(error), progress, remaining, max_voltage,
        params.min_speed, config_.trapezoid.angular_accel_deg,
        config_.trapezoid.angular_decel_deg);

    tank_voltage(turn, -turn);
    motion_progress_ = progress;

    if (exit.settled(std::fabs(error))) return true;
    pros::delay(config_.loop_delay_ms);
  }

  return false;
}

bool Chassis::run_swing_to_heading(double heading_deg, DriveSide locked_side,
                                   int timeout_ms,
                                   const SwingToHeadingParams& params) {
  if (odom_ == nullptr) return false;

  swing_pid_.reset();
  const std::uint32_t start_ms = pros::millis();
  const int max_voltage = speed_to_voltage(params.max_speed);
  const double initial_error =
      std::fabs(directed_error(heading_deg, odom_->heading_deg(),
                               params.direction));
  ExitCondition exit;
  exit.small_error = params.early_exit_range_deg > 0.0
                         ? params.early_exit_range_deg
                         : config_.turn_settle_error_deg;
  exit.small_time_ms = config_.settle_time_ms;
  exit.big_error = exit.small_error * 2.0;
  exit.big_time_ms = config_.settle_time_ms * 2;

  while (!timed_out(start_ms, timeout_ms) && !motion_cancel_) {
    const Pose current = odom_->get_pose();
    const double error =
        directed_error(heading_deg, current.heading_deg(), params.direction);
    const double remaining = std::fabs(error);
    const double progress = clamp_double(initial_error - remaining, 0.0,
                                         initial_error);
    int turn = apply_trapezoid(
        swing_pid_.step(error), progress, remaining, max_voltage,
        params.min_speed, config_.trapezoid.angular_accel_deg,
        config_.trapezoid.angular_decel_deg);

    if (locked_side == DriveSide::LEFT) {
      tank_voltage(0, -turn);
    } else {
      tank_voltage(turn, 0);
    }

    motion_progress_ = progress;
    if (exit.settled(std::fabs(error))) return true;
    pros::delay(config_.loop_delay_ms);
  }

  return false;
}

bool Chassis::run_move_to_point(double x_in, double y_in, int timeout_ms,
                                const MoveToPointParams& params) {
  if (odom_ == nullptr) return false;

  point_drive_pid_.reset();
  point_turn_pid_.reset();
  SlewLimiter slew(config_.slew_step_mv);
  const std::uint32_t start_ms = pros::millis();
  const double start_x = odom_->x();
  const double start_y = odom_->y();
  const double total_distance = distance_between(start_x, start_y, x_in, y_in);
  const int max_voltage = speed_to_voltage(params.max_speed);
  ExitCondition exit;
  exit.small_error = params.early_exit_range_in > 0.0
                         ? params.early_exit_range_in
                         : config_.drive_settle_error_in;
  exit.small_time_ms = config_.settle_time_ms;
  exit.big_error = exit.small_error * 2.0;
  exit.big_time_ms = config_.settle_time_ms * 2;

  while (!timed_out(start_ms, timeout_ms) && !motion_cancel_) {
    const Pose current = odom_->get_pose();
    const double dist = distance_between(current.x, current.y, x_in, y_in);
    double target_heading = bearing_to_point_deg(current.x, current.y, x_in, y_in);
    double drive_direction = 1.0;
    if (!params.forwards) {
      target_heading += 180.0;
      drive_direction = -1.0;
    }

    const double heading_error =
        angle_error_deg(target_heading, current.heading_deg());
    const double progress = clamp_double(
        distance_between(start_x, start_y, current.x, current.y), 0.0,
        total_distance);
    const int profile_limit = trapezoid_limit(
        progress, dist, max_voltage, params.min_speed,
        config_.trapezoid.linear_accel_in, config_.trapezoid.linear_decel_in);
    int drive = apply_trapezoid(
        point_drive_pid_.step(dist * drive_direction), progress, dist,
        max_voltage, params.min_speed, config_.trapezoid.linear_accel_in,
        config_.trapezoid.linear_decel_in);
    int turn = clamp_voltage(point_turn_pid_.step(heading_error), max_voltage);
    if (params.slew) drive = static_cast<int>(slew.step(drive));

    tank_drive_turn_profiled(drive, turn, profile_limit);
    motion_progress_ = progress;

    if (exit.settled(dist)) return true;
    pros::delay(config_.loop_delay_ms);
  }

  return false;
}

bool Chassis::run_move_to_pose(double x_in, double y_in, double theta_deg,
                               int timeout_ms,
                               const MoveToPoseParams& params) {
  if (odom_ == nullptr) return false;

  pose_drive_pid_.reset();
  pose_turn_pid_.reset();
  SlewLimiter slew(config_.slew_step_mv);
  const std::uint32_t start_ms = pros::millis();
  const double start_x = odom_->x();
  const double start_y = odom_->y();
  const double total_distance = distance_between(start_x, start_y, x_in, y_in);
  const int max_voltage = speed_to_voltage(params.max_speed);
  ExitCondition exit;
  exit.small_error = params.early_exit_range_in > 0.0
                         ? params.early_exit_range_in
                         : config_.drive_settle_error_in;
  exit.small_time_ms = config_.settle_time_ms;
  exit.big_error = exit.small_error * 2.0;
  exit.big_time_ms = config_.settle_time_ms * 2;

  while (!timed_out(start_ms, timeout_ms) && !motion_cancel_) {
    const Pose current = odom_->get_pose();
    const double dist = distance_between(current.x, current.y, x_in, y_in);
    const double final_heading_rad = deg_to_rad(theta_deg);
    const double lead = clamp_double(dist * params.lead, 0.0, params.max_lead_in);
    double carrot_x = x_in - std::sin(final_heading_rad) * lead;
    double carrot_y = y_in - std::cos(final_heading_rad) * lead;
    if (dist < std::max(6.0, exit.small_error * 3.0)) {
      carrot_x = x_in;
      carrot_y = y_in;
    }

    const double carrot_bearing =
        bearing_to_point_deg(current.x, current.y, carrot_x, carrot_y);
    const double close_t = 1.0 - clamp_double(dist / 24.0, 0.0, 1.0);
    const double turn_blend =
        clamp_double(params.turn_weight + close_t * (1.0 - params.turn_weight),
                     0.0, 1.0);

    double desired_heading =
        interpolate_heading(carrot_bearing, theta_deg, turn_blend);
    double drive_direction = 1.0;
    if (!params.forwards) {
      desired_heading += 180.0;
      drive_direction = -1.0;
    }

    const double heading_error =
        angle_error_deg(desired_heading, current.heading_deg());
    const double final_heading_error =
        std::fabs(angle_error_deg(theta_deg, current.heading_deg()));
    const double progress = clamp_double(
        distance_between(start_x, start_y, current.x, current.y), 0.0,
        total_distance);
    const int profile_limit = trapezoid_limit(
        progress, dist, max_voltage, params.min_speed,
        config_.trapezoid.linear_accel_in, config_.trapezoid.linear_decel_in);
    int drive = apply_trapezoid(
        pose_drive_pid_.step(dist * drive_direction), progress, dist,
        max_voltage, params.min_speed, config_.trapezoid.linear_accel_in,
        config_.trapezoid.linear_decel_in);
    int turn = clamp_voltage(pose_turn_pid_.step(heading_error), max_voltage);
    if (params.slew) drive = static_cast<int>(slew.step(drive));

    tank_drive_turn_profiled(drive, turn, profile_limit);
    motion_progress_ = progress;

    const bool heading_ok =
        params.early_exit_heading_deg <= 0.0 ||
        final_heading_error <= params.early_exit_heading_deg;
    if (exit.settled(dist) && heading_ok) return true;
    pros::delay(config_.loop_delay_ms);
  }

  return false;
}

bool Chassis::run_follow(Path path, int timeout_ms, const FollowParams& params) {
  if (odom_ == nullptr || path.size() < 2) return false;

  if (params.smooth) {
    path = smooth_path(inject_path(path, params.inject_spacing_in),
                       params.smooth_weight_data, params.smooth_weight_smooth);
  }

  std::vector<double> cumulative(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    cumulative[i] = cumulative[i - 1] +
                    distance_between(path[i - 1].x, path[i - 1].y, path[i].x,
                                     path[i].y);
  }

  follow_drive_pid_.reset();
  follow_turn_pid_.reset();
  const std::uint32_t start_ms = pros::millis();
  const int max_voltage = speed_to_voltage(params.max_speed);
  const double total_length = cumulative.back();

  while (!timed_out(start_ms, timeout_ms) && !motion_cancel_) {
    const Pose current = odom_->get_pose();
    std::size_t closest = 0;
    double closest_dist = distance_between(current.x, current.y, path[0].x,
                                           path[0].y);

    for (std::size_t i = 1; i < path.size(); ++i) {
      const double dist =
          distance_between(current.x, current.y, path[i].x, path[i].y);
      if (dist < closest_dist) {
        closest_dist = dist;
        closest = i;
      }
    }

    std::size_t target_index = closest;
    for (std::size_t i = closest; i < path.size(); ++i) {
      if (distance_between(current.x, current.y, path[i].x, path[i].y) >=
          params.lookahead_in) {
        target_index = i;
        break;
      }
      target_index = i;
    }

    const PathPoint& target = path[target_index];
    const PathPoint& end = path.back();
    set_target("Follow", target.x, target.y);
    const double end_dist = distance_between(current.x, current.y, end.x, end.y);
    const double path_progress = clamp_double(cumulative[closest], 0.0,
                                              total_length);
    const double path_remaining = std::max(0.0, total_length - path_progress);
    double target_heading =
        bearing_to_point_deg(current.x, current.y, target.x, target.y);
    double drive_direction = 1.0;
    if (!params.forwards) {
      target_heading += 180.0;
      drive_direction = -1.0;
    }

    const int target_limit =
        std::min(max_voltage, speed_to_voltage(target.speed));
    const double heading_error =
        angle_error_deg(target_heading, current.heading_deg());
    const int profile_limit = trapezoid_limit(
        path_progress, path_remaining, target_limit, params.min_speed,
        config_.trapezoid.linear_accel_in, config_.trapezoid.linear_decel_in);
    int drive = apply_trapezoid(
        follow_drive_pid_.step(end_dist * drive_direction), path_progress,
        path_remaining, target_limit, params.min_speed,
        config_.trapezoid.linear_accel_in, config_.trapezoid.linear_decel_in);
    int turn =
        clamp_voltage(follow_turn_pid_.step(heading_error), target_limit);

    tank_drive_turn_profiled(drive, turn, profile_limit);
    motion_progress_ = path_progress;

    if (end_dist <= params.early_exit_range_in) return true;
    pros::delay(config_.loop_delay_ms);
  }

  return false;
}

int Chassis::clamp_voltage(double voltage, int max_voltage) const {
  const int limit = clamp_int(max_voltage, 0, 12000);
  if (voltage > limit) return limit;
  if (voltage < -limit) return -limit;
  return static_cast<int>(voltage);
}

int Chassis::speed_to_voltage(int speed) const {
  return clamp_int(speed, 0, 127) * 12000 / 127;
}

int Chassis::voltage_to_speed(int voltage) const {
  return clamp_int(std::abs(voltage), 0, 12000) * 127 / 12000;
}

int Chassis::apply_min_speed(int voltage, int min_speed) const {
  if (min_speed <= 0 || voltage == 0) return voltage;
  const int min_voltage = speed_to_voltage(min_speed);
  if (std::abs(voltage) >= min_voltage) return voltage;
  return sign_of(voltage) * min_voltage;
}

int Chassis::trapezoid_limit(double progress, double remaining, int max_voltage,
                             int min_speed, double accel_distance,
                             double decel_distance) const {
  const int clipped_max = clamp_int(max_voltage, 0, 12000);
  if (!config_.trapezoid.enabled || clipped_max == 0) return clipped_max;

  const int profile_min_speed =
      std::max(clamp_int(min_speed, 0, 127),
               clamp_int(config_.trapezoid.min_speed, 0, 127));
  const int min_voltage = std::min(speed_to_voltage(profile_min_speed),
                                   clipped_max);

  const double accel_ratio = accel_distance <= 0.0
                                 ? 1.0
                                 : clamp_double(progress / accel_distance,
                                                0.0, 1.0);
  const double decel_ratio = decel_distance <= 0.0
                                 ? 1.0
                                 : clamp_double(remaining / decel_distance,
                                                0.0, 1.0);
  const double ratio = std::min(accel_ratio, decel_ratio);
  const double voltage =
      min_voltage + (clipped_max - min_voltage) * clamp_double(ratio, 0.0, 1.0);

  return clamp_int(static_cast<int>(voltage), min_voltage, clipped_max);
}

int Chassis::apply_trapezoid(double voltage, double progress, double remaining,
                             int max_voltage, int min_speed,
                             double accel_distance,
                             double decel_distance) const {
  const int limit = trapezoid_limit(progress, remaining, max_voltage, min_speed,
                                    accel_distance, decel_distance);
  const int clipped = clamp_voltage(voltage, limit);
  if (voltage == 0.0) return clipped;

  const int profile_min_speed =
      std::max(clamp_int(min_speed, 0, 127),
               clamp_int(config_.trapezoid.min_speed, 0, 127));
  return clamp_voltage(apply_min_speed(clipped, profile_min_speed), limit);
}

void Chassis::tank_drive_turn_profiled(int drive_mv, int turn_mv,
                                       int side_limit_mv) {
  double left = drive_mv + turn_mv;
  double right = drive_mv - turn_mv;
  const double limit = std::max(0, side_limit_mv);
  const double max_mag = std::max(std::fabs(left), std::fabs(right));

  if (limit > 0.0 && max_mag > limit) {
    const double scale = limit / max_mag;
    left *= scale;
    right *= scale;
  }

  tank_voltage(static_cast<int>(left), static_cast<int>(right));
}

bool Chassis::timed_out(std::uint32_t start_ms,
                        std::uint32_t timeout_ms) const {
  return pros::millis() - start_ms >= timeout_ms;
}

bool Chassis::safety_ok(int commanded_left_mv, int commanded_right_mv) {
  safety_message_ = "ok";

  if (safety_config_.stop_on_over_current &&
      (group_over_current(left_) || group_over_current(right_))) {
    safety_message_ = "drive over current";
    motion_cancel_ = true;
    return false;
  }

  if (safety_config_.stop_on_over_temp &&
      (group_over_temp(left_) || group_over_temp(right_))) {
    safety_message_ = "drive over temp";
    motion_cancel_ = true;
    return false;
  }

  if (safety_config_.stop_on_stall &&
      (group_stalled(left_, commanded_left_mv) ||
       group_stalled(right_, commanded_right_mv))) {
    if (stall_since_ms_ == 0) stall_since_ms_ = pros::millis();
    if (pros::millis() - stall_since_ms_ >= safety_config_.stall_time_ms) {
      safety_message_ = "drive stalled";
      motion_cancel_ = true;
      return false;
    }
  } else {
    stall_since_ms_ = 0;
  }

  return true;
}

bool Chassis::group_over_current(pros::MotorGroup* group) const {
  if (group == nullptr) return false;
#if __cplusplus >= 202002L
  const std::vector<std::int32_t> flags = group->is_over_current_all();
#else
  const std::vector<std::int32_t> flags = group->are_over_current();
#endif
  for (std::int32_t flag : flags) {
    if (flag) return true;
  }
  return false;
}

bool Chassis::group_over_temp(pros::MotorGroup* group) const {
  if (group == nullptr) return false;
#if __cplusplus >= 202002L
  const std::vector<std::int32_t> flags = group->is_over_temp_all();
#else
  const std::vector<std::int32_t> flags = group->are_over_temp();
#endif
  for (std::int32_t flag : flags) {
    if (flag) return true;
  }
  return false;
}

bool Chassis::group_stalled(pros::MotorGroup* group, int command_mv) const {
  if (group == nullptr || std::abs(command_mv) < 5000) return false;

#if __cplusplus >= 202002L
  const std::vector<double> velocities = group->get_actual_velocity_all();
  const std::vector<std::int32_t> currents = group->get_current_draw_all();
#else
  const std::vector<double> velocities = group->get_actual_velocities();
  const std::vector<std::int32_t> currents = group->get_current_draws();
#endif

  if (velocities.empty() || currents.empty()) return false;

  double velocity_sum = 0.0;
  for (double velocity : velocities) velocity_sum += std::fabs(velocity);
  const double avg_velocity = velocity_sum / velocities.size();

  double current_sum = 0.0;
  for (std::int32_t current : currents) current_sum += current;
  const double avg_current = current_sum / currents.size();

  return avg_velocity <= safety_config_.stall_velocity_rpm &&
         avg_current >= safety_config_.stall_current_ma;
}

void Chassis::set_target(const char* type, double x, double y, double theta,
                         double distance) {
  current_target_.valid = type != nullptr && type[0] != '\0' &&
                          std::string(type) != "Idle";
  current_target_.type = type == nullptr ? "Idle" : type;
  current_target_.x = x;
  current_target_.y = y;
  current_target_.theta = theta;
  current_target_.distance = distance;
}

double Chassis::directed_error(double target_deg, double current_deg,
                               AngularDirection direction) const {
  double error = angle_error_deg(target_deg, current_deg);

  if (direction == AngularDirection::CW_CLOCKWISE && error < 0.0) {
    error += 360.0;
  } else if (direction == AngularDirection::CCW_COUNTERCLOCKWISE &&
             error > 0.0) {
    error -= 360.0;
  }

  return error;
}

double Chassis::interpolate_heading(double from_deg, double to_deg,
                                    double t) const {
  return from_deg + angle_error_deg(to_deg, from_deg) *
                        clamp_double(t, 0.0, 1.0);
}

// ===========================================================================
// LemLib & EZ-Template Compatibility Layer (Highly User Friendly API)
// ===========================================================================

void Chassis::driveDistance(double distance_in, int timeout_ms, int max_voltage, bool slew, bool async) {
  MotionCommand command;
  command.type = MotionType::DRIVE_DISTANCE;
  command.distance = distance_in;
  command.timeout_ms = timeout_ms;
  command.max_voltage = max_voltage;
  command.slew = slew;
  dispatch(command, async);
}

void Chassis::calibrate() {
  if (odom_ != nullptr) {
    odom_->calibrate();
  }
}

Pose Chassis::getPose() const {
  return odom_ ? odom_->get_pose() : Pose();
}

void Chassis::setPose(double x, double y, double heading) {
  if (odom_ != nullptr) {
    odom_->set_pose(x, y, heading);
  }
}

void Chassis::pid_wait() {
  waitUntilDone();
}

void Chassis::pid_wait_until(double progress) {
  waitUntil(progress);
}

void Chassis::drive_angle_set(double heading_deg) {
  if (odom_ != nullptr) {
    odom_->set_heading_deg(heading_deg);
  }
}

void Chassis::odom_xyt_set(double x_in, double y_in, double heading_deg) {
  if (odom_ != nullptr) {
    odom_->set_pose(x_in, y_in, heading_deg);
  }
}

void Chassis::pid_drive_set(double distance_in, int max_speed, bool slew) {
  driveDistance(distance_in, 3000, speed_to_voltage(max_speed), slew, true);
}

void Chassis::pid_turn_set(double heading_deg, int max_speed, bool slew) {
  TurnToHeadingParams params;
  params.max_speed = max_speed;
  turnToHeading(heading_deg, 2000, params, true);
}

void Chassis::pid_turn_relative_set(double delta_deg, int max_speed, bool slew) {
  if (odom_ != nullptr) {
    double target = odom_->heading_deg() + delta_deg;
    pid_turn_set(target, max_speed, slew);
  }
}

void Chassis::pid_swing_set(DriveSide swing_side, double heading_deg, int max_speed,
                            int opposite_side_speed, bool slew) {
  SwingToHeadingParams params;
  params.max_speed = max_speed;
  
  // LEFT_SWING (swing_side = LEFT) means Left side moves, so Right side is locked.
  // RIGHT_SWING (swing_side = RIGHT) means Right side moves, so Left side is locked.
  DriveSide locked = (swing_side == DriveSide::LEFT) ? DriveSide::RIGHT : DriveSide::LEFT;
  swingToHeading(heading_deg, locked, 2000, params, true);
}

void Chassis::pid_swing_relative_set(DriveSide swing_side, double delta_deg, int max_speed,
                                     int opposite_side_speed, bool slew) {
  if (odom_ != nullptr) {
    double target = odom_->heading_deg() + delta_deg;
    pid_swing_set(swing_side, target, max_speed, opposite_side_speed, slew);
  }
}

void Chassis::pid_odom_set(double x_in, double y_in, int max_speed, bool slew) {
  MoveToPointParams params;
  params.max_speed = max_speed;
  params.slew = slew;
  moveToPoint(x_in, y_in, 4000, params, true);
}

void Chassis::pid_odom_set(double x_in, double y_in, double heading_deg, int max_speed, bool slew) {
  MoveToPoseParams params;
  params.max_speed = max_speed;
  params.slew = slew;
  moveToPose(x_in, y_in, heading_deg, 4000, params, true);
}

void Chassis::pid_turn_set(std::pair<double, double> target_point, ::ez::DriveDirection dir, int max_speed) {
  TurnToPointParams params;
  params.max_speed = max_speed;
  params.forwards = (dir == ::ez::fwd);
  turnToPoint(target_point.first, target_point.second, 2000, params, true);
}

}  // namespace Vortex
