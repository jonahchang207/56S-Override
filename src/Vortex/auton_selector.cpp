#include "auton_selector.hpp"

#include "chassis.hpp"
#include "odom.hpp"

#include <algorithm>
#include <cstdio>

namespace Vortex {

namespace {
int clamp_int_local(int value, int low, int high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}
}  // namespace

AutonSelector& AutonSelector::add(const char* name, AutonCallback callback) {
  routines_.push_back(AutonRoutine(name, callback));
  if (selected_ >= static_cast<int>(routines_.size())) selected_ = 0;
  return *this;
}

AutonSelector& AutonSelector::add_many(
    std::initializer_list<AutonRoutine> routines) {
  for (const AutonRoutine& routine : routines) {
    add(routine.name, routine.callback);
  }
  return *this;
}

void AutonSelector::clear() {
  routines_.clear();
  selected_ = 0;
}

void AutonSelector::next() {
  if (routines_.empty()) return;
  selected_ = (selected_ + 1) % routines_.size();
  status_ = "Changed";
  print();
}

void AutonSelector::previous() {
  if (routines_.empty()) return;
  selected_--;
  if (selected_ < 0) selected_ = routines_.size() - 1;
  status_ = "Changed";
  print();
}

void AutonSelector::set_selected(int index) {
  if (routines_.empty()) {
    selected_ = 0;
    return;
  }

  if (index < 0) index = 0;
  if (index >= static_cast<int>(routines_.size())) {
    index = routines_.size() - 1;
  }
  selected_ = index;
  print();
}

int AutonSelector::selected_index() const { return selected_; }

const char* AutonSelector::selected_name() const {
  if (routines_.empty()) return "None";
  return routines_[selected_].name;
}

int AutonSelector::size() const { return routines_.size(); }

void AutonSelector::bind_dashboard(Odometry* odom, Chassis* chassis,
                                   pros::Controller* controller) {
  odom_ = odom;
  chassis_ = chassis;
  controller_ = controller;
}

void AutonSelector::set_dashboard_config(const AutonDashboardConfig& config) {
  dashboard_config_ = config;
}

AutonDashboardConfig AutonSelector::dashboard_config() const {
  return dashboard_config_;
}

void AutonSelector::start_dashboard() {
  if (dashboard_task_ != nullptr) return;

  if (!pros::lcd::is_initialized()) pros::lcd::initialize();
  pros::lcd::clear();
  dashboard_running_ = true;
  dashboard_task_ = new pros::Task(dashboard_entry, this, TASK_PRIORITY_DEFAULT,
                                   TASK_STACK_DEPTH_DEFAULT, "auton_dash");
}

void AutonSelector::stop_dashboard() {
  dashboard_running_ = false;
  if (dashboard_task_ == nullptr) return;

  dashboard_task_->remove();
  delete dashboard_task_;
  dashboard_task_ = nullptr;
}

void AutonSelector::render_dashboard() {
  lcd_line(0, selected_line());
  lcd_line(1, clip(std::string("Auton: ") + selected_name()));
  lcd_line(2, odom_line());
  lcd_line(3, target_line());
  lcd_line(4, battery_line());
  lcd_line(5, clip(std::string("LCD < > choose  C save  ") + status_));
  lcd_line(6, temp_values_line());
  lcd_line(7, temp_bar_line());
}

void AutonSelector::set_status(const char* status) {
  status_ = status == nullptr ? "" : status;
}

void AutonSelector::run_selected() const {
  if (routines_.empty() || routines_[selected_].callback == nullptr) return;
  routines_[selected_].callback();
}

void AutonSelector::print() const {
  std::printf("[auton] %d/%d: %s\n", selected_ + 1, size(), selected_name());
}

void AutonSelector::handle_controller(pros::Controller& controller,
                                      bool run_on_a) {
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    next();
    status_ = save_selected() ? "Saved" : "Save failed";
  }
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    previous();
    status_ = save_selected() ? "Saved" : "Save failed";
  }
  if (run_on_a && controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    run_selected();
  }
}

bool AutonSelector::save_selected(const char* filename) const {
  std::FILE* file = std::fopen(file_or_default(filename), "w");
  if (file == nullptr) return false;

  std::fprintf(file, "%d\n", selected_);
  std::fclose(file);
  return true;
}

bool AutonSelector::load_selected(const char* filename) {
  std::FILE* file = std::fopen(file_or_default(filename), "r");
  if (file == nullptr) return false;

  int index = 0;
  const int count = std::fscanf(file, "%d", &index);
  std::fclose(file);
  if (count != 1) return false;

  set_selected(index);
  status_ = "Loaded";
  return true;
}

void AutonSelector::dashboard_entry(void* params) {
  AutonSelector* selector = static_cast<AutonSelector*>(params);
  while (selector != nullptr && selector->dashboard_running_) {
    selector->handle_lcd_buttons();
    selector->render_dashboard();
    pros::delay(selector->dashboard_config_.refresh_ms);
  }
}

void AutonSelector::handle_lcd_buttons() {
  const std::uint32_t buttons = pros::lcd::read_buttons();
  const std::uint32_t pressed = buttons & ~last_lcd_buttons_;
  last_lcd_buttons_ = buttons;

  if (pressed & LCD_BTN_LEFT) {
    previous();
    save_selected();
    status_ = "Saved";
  }
  if (pressed & LCD_BTN_RIGHT) {
    next();
    save_selected();
    status_ = "Saved";
  }
  if (pressed & LCD_BTN_CENTER) {
    status_ = save_selected() ? "Saved" : "Save failed";
  }
}

void AutonSelector::lcd_line(int line, const std::string& text) const {
  pros::lcd::set_text(line, clip(text));
}

std::string AutonSelector::selected_line() const {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "< %d/%d > %s", selected_ + 1, size(),
                selected_name());
  return clip(buffer);
}

std::string AutonSelector::odom_line() const {
  if (odom_ == nullptr) return "Odom: not bound";

  const Pose pose = odom_->get_pose();
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "O X:%5.1f Y:%5.1f H:%5.1f",
                pose.x, pose.y, pose.heading_deg());
  return clip(buffer);
}

std::string AutonSelector::target_line() const {
  if (chassis_ == nullptr) return "Target: chassis not bound";

  const Chassis::MotionTarget target = chassis_->currentTarget();
  if (!target.valid) {
    return clip(std::string("Next: ") + selected_name());
  }

  char buffer[80];
  if (std::string(target.type) == "Drive") {
    std::snprintf(buffer, sizeof(buffer), "Target Drive %.1fin",
                  target.distance);
  } else if (std::string(target.type) == "Turn" ||
             std::string(target.type) == "Swing") {
    std::snprintf(buffer, sizeof(buffer), "Target %s %.1fdeg", target.type,
                  target.theta);
  } else {
    std::snprintf(buffer, sizeof(buffer), "Target %s X%.1f Y%.1f H%.1f",
                  target.type, target.x, target.y, target.theta);
  }

  return clip(buffer);
}

std::string AutonSelector::battery_line() const {
  const double brain = pros::battery::get_capacity();
  const int controller =
      controller_ == nullptr ? -1 : controller_->get_battery_capacity();

  char buffer[64];
  if (controller >= 0) {
    std::snprintf(buffer, sizeof(buffer), "Batt Brain:%3.0f%% Ctrl:%3d%%",
                  brain, controller);
  } else {
    std::snprintf(buffer, sizeof(buffer), "Batt Brain:%3.0f%%", brain);
  }
  return clip(buffer);
}

std::string AutonSelector::temp_values_line() const {
  if (chassis_ == nullptr) return "TempC: chassis not bound";

  const std::vector<MotorTemperature> temps = chassis_->motorTemperatures();
  if (temps.empty()) return "TempC: no drive ports";

  const int per_page = std::max(1, dashboard_config_.temps_per_page);
  const int pages = (temps.size() + per_page - 1) / per_page;
  const std::uint32_t page_ms =
      dashboard_config_.temp_page_ms == 0 ? 1 : dashboard_config_.temp_page_ms;
  const int page =
      pages <= 1 ? 0
                 : (pros::millis() / page_ms) % pages;
  const int start = page * per_page;
  const int end = std::min(start + per_page, static_cast<int>(temps.size()));

  std::string line = "TempC";
  for (int i = start; i < end; ++i) {
    char part[20];
    std::snprintf(part, sizeof(part), " %s:%2.0f", temps[i].tag.c_str(),
                  temps[i].celsius);
    line += part;
  }
  return clip(line);
}

std::string AutonSelector::temp_bar_line() const {
  if (chassis_ == nullptr) return "";

  const std::vector<MotorTemperature> temps = chassis_->motorTemperatures();
  if (temps.empty()) return "";

  const int per_page = std::max(1, dashboard_config_.temps_per_page);
  const int pages = (temps.size() + per_page - 1) / per_page;
  const std::uint32_t page_ms =
      dashboard_config_.temp_page_ms == 0 ? 1 : dashboard_config_.temp_page_ms;
  const int page =
      pages <= 1 ? 0
                 : (pros::millis() / page_ms) % pages;
  const int start = page * per_page;
  const int end = std::min(start + per_page, static_cast<int>(temps.size()));

  std::string line;
  for (int i = start; i < end; ++i) {
    if (!line.empty()) line += " ";
    line += temps[i].tag + temp_bar(temps[i].percent);
  }
  return clip(line);
}

std::string AutonSelector::temp_bar(int percent) const {
  const int fill = clamp_int_local(percent / 25, 0, 4);
  std::string bar = "[";
  for (int i = 0; i < 4; ++i) {
    bar += i < fill ? '#' : '-';
  }
  bar += "]";
  return bar;
}

std::string AutonSelector::clip(const std::string& text,
                                std::size_t max_len) const {
  if (text.size() <= max_len) return text;
  return text.substr(0, max_len);
}

const char* AutonSelector::file_or_default(const char* filename) const {
  if (filename != nullptr) return filename;
  return dashboard_config_.save_file;
}

}  // namespace Vortex
