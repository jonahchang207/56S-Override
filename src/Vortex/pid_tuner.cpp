#include "pid_tuner.hpp"

#include <cstdio>

namespace Vortex {

void PIDTuner::bind_chassis(Chassis* chassis) { chassis_ = chassis; }

void PIDTuner::bind_defaults() {
  if (chassis_ == nullptr) return;

  MotionConfig* config = chassis_->motion_config();
  add("Drive", &config->drive);
  add("Turn", &config->turn);
  add("Swing", &config->swing);
  add("Point Drive", &config->point_drive);
  add("Point Turn", &config->point_turn);
  add("Pose Drive", &config->pose_drive);
  add("Pose Turn", &config->pose_turn);
  add("Follow Drive", &config->follow_drive);
  add("Follow Turn", &config->follow_turn);
}

void PIDTuner::add(const char* name, PIDGains* gains) {
  entries_.push_back({name, gains});
}

void PIDTuner::clear() {
  entries_.clear();
  selected_entry_ = 0;
  selected_field_ = 0;
}

void PIDTuner::enable() {
  enabled_ = true;
  print();
}

void PIDTuner::disable() { enabled_ = false; }

void PIDTuner::toggle() {
  enabled_ = !enabled_;
  if (enabled_) print();
}

bool PIDTuner::enabled() const { return enabled_; }

void PIDTuner::set_print_terminal(bool enabled) { print_terminal_ = enabled; }

bool PIDTuner::print_terminal_enabled() const { return print_terminal_; }

void PIDTuner::set_increments(double p, double i, double d, double zone,
                              double max_i) {
  inc_p_ = p;
  inc_i_ = i;
  inc_d_ = d;
  inc_zone_ = zone;
  inc_max_i_ = max_i;
}

void PIDTuner::iterate(pros::Controller& controller) {
  if (!enabled_ || entries_.empty()) return;

  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    selected_entry_ = (selected_entry_ + 1) % entries_.size();
    print();
  }
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    selected_entry_--;
    if (selected_entry_ < 0) selected_entry_ = entries_.size() - 1;
    print();
  }
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    selected_field_--;
    if (selected_field_ < 0) selected_field_ = 4;
    print();
  }
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
    selected_field_ = (selected_field_ + 1) % 5;
    print();
  }
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    switch (selected_field_) {
      case 0: adjust(inc_p_); break;
      case 1: adjust(inc_i_); break;
      case 2: adjust(inc_d_); break;
      case 3: adjust(inc_zone_); break;
      case 4: adjust(inc_max_i_); break;
    }
  }
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
    switch (selected_field_) {
      case 0: adjust(-inc_p_); break;
      case 1: adjust(-inc_i_); break;
      case 2: adjust(-inc_d_); break;
      case 3: adjust(-inc_zone_); break;
      case 4: adjust(-inc_max_i_); break;
    }
  }
}

void PIDTuner::print() const {
  if (!print_terminal_ || entries_.empty()) return;

  const Entry& entry = entries_[selected_entry_];
  if (entry.gains == nullptr) return;

  std::printf("[tuner] %s %s = %.4f | P %.4f I %.4f D %.4f Z %.4f MAXI %.4f\n",
              entry.name, field_name(), field_value(*entry.gains),
              entry.gains->kP, entry.gains->kI, entry.gains->kD,
              entry.gains->integral_zone, entry.gains->max_integral);
}

void PIDTuner::adjust(double amount) {
  if (entries_.empty()) return;
  PIDGains* gains = entries_[selected_entry_].gains;
  if (gains == nullptr) return;

  switch (selected_field_) {
    case 0: gains->kP += amount; break;
    case 1: gains->kI += amount; break;
    case 2: gains->kD += amount; break;
    case 3: gains->integral_zone += amount; break;
    case 4: gains->max_integral += amount; break;
  }

  if (gains->integral_zone < 0.0) gains->integral_zone = 0.0;
  if (gains->max_integral < 0.0) gains->max_integral = 0.0;
  mark_changed();
  print();
}

void PIDTuner::mark_changed() {
  if (chassis_ != nullptr) {
    chassis_->set_motion_config(*chassis_->motion_config());
  }
}

const char* PIDTuner::field_name() const {
  switch (selected_field_) {
    case 0: return "P";
    case 1: return "I";
    case 2: return "D";
    case 3: return "zone";
    case 4: return "maxI";
    default: return "?";
  }
}

double PIDTuner::field_value(const PIDGains& gains) const {
  switch (selected_field_) {
    case 0: return gains.kP;
    case 1: return gains.kI;
    case 2: return gains.kD;
    case 3: return gains.integral_zone;
    case 4: return gains.max_integral;
    default: return 0.0;
  }
}

}  // namespace Vortex
