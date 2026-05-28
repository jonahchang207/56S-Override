#pragma once

/**
 * @file auton_selector.hpp
 * @brief LCD / controller-driven autonomous routine selector.
 *
 * AutonSelector lets you register named autonomous functions, cycle through
 * them on the V5 brain LCD or controller, persist the choice to a file on
 * the SD card, and optionally render a live dashboard (pose, target, battery,
 * motor temperatures) while the robot is idle.
 */

#include "api.h"

#include <cstdint>
#include <initializer_list>
#include <string>
#include <vector>

namespace Vortex {

class Chassis;
class Odometry;

/** @brief Function pointer signature for an autonomous routine. */
typedef void (*AutonCallback)();

/**
 * @struct AutonRoutine
 * @brief A named (display name + callback) pair shown in the selector list.
 */
struct AutonRoutine {
  const char* name = "";            ///< Display name on the LCD/controller.
  AutonCallback callback = nullptr; ///< Function called by run_selected().

  /** @brief Default empty routine. */
  AutonRoutine() = default;

  /**
   * @param iname     Display name (must outlive the AutonRoutine).
   * @param icallback Function to invoke.
   */
  AutonRoutine(const char* iname, AutonCallback icallback)
      : name(iname), callback(icallback) {}
};

/**
 * @struct AutonDashboardConfig
 * @brief Tuning for the live dashboard task.
 */
struct AutonDashboardConfig {
  std::uint32_t refresh_ms = 150;          ///< LCD refresh period.
  std::uint32_t temp_page_ms = 1800;       ///< Time per motor-temp page.
  const char* save_file = "/usd/auton.txt"; ///< File used for save/load.
  int temps_per_page = 3;                   ///< Motors shown per temp page.
};

/**
 * @class AutonSelector
 * @brief Manages a list of autonomous routines and a small UI to pick one.
 *
 * Typical use:
 * @code
 * auton_selector.add("Red left",  red_left_auton)
 *               .add("Blue right", blue_right_auton);
 * auton_selector.bind_dashboard(&odom, &chassis, &master);
 * auton_selector.load_selected();
 * auton_selector.start_dashboard();
 * @endcode
 */
class AutonSelector {
 public:
  /**
   * @brief Append a routine to the list.
   * @return Reference to this for chaining.
   */
  AutonSelector& add(const char* name, AutonCallback callback);

  /**
   * @brief Bulk-add routines.
   * @return Reference to this for chaining.
   */
  AutonSelector& add_many(std::initializer_list<AutonRoutine> routines);

  /** @brief Remove every registered routine. */
  void clear();

  /** @brief Advance the selection by one (wraps). */
  void next();

  /** @brief Move the selection back by one (wraps). */
  void previous();

  /** @brief Jump directly to a routine by index. */
  void set_selected(int index);

  /** @brief Index of the currently selected routine. */
  int selected_index() const;

  /** @brief Name of the currently selected routine. */
  const char* selected_name() const;

  /** @brief Number of registered routines. */
  int size() const;

  /**
   * @brief Hand the selector references it needs for dashboard rendering.
   * @param odom        Pose source for the "pose" row.
   * @param chassis     Motion driver for target + temperature rows.
   * @param controller  Optional controller for the secondary UI.
   */
  void bind_dashboard(Odometry* odom, Chassis* chassis,
                      pros::Controller* controller = nullptr);

  /** @brief Replace the dashboard configuration. */
  void set_dashboard_config(const AutonDashboardConfig& config);

  /** @brief Snapshot of the dashboard configuration. */
  AutonDashboardConfig dashboard_config() const;

  /** @brief Spawn the background dashboard task. */
  void start_dashboard();

  /** @brief Stop the background dashboard task. */
  void stop_dashboard();

  /** @brief Render one frame of the dashboard now (no task needed). */
  void render_dashboard();

  /**
   * @brief Override the status string shown in the dashboard.
   * @param status Static C string (must outlive the selector).
   */
  void set_status(const char* status);

  /** @brief Invoke the currently selected routine. */
  void run_selected() const;

  /** @brief Print the list and current selection to stdout. */
  void print() const;

  /**
   * @brief Drive selection from a controller during opcontrol.
   * @param controller Source of button presses.
   * @param run_on_a   If true, pressing A also runs the selected routine.
   */
  void handle_controller(pros::Controller& controller, bool run_on_a = false);

  /**
   * @brief Persist the current selection to disk.
   * @param filename Optional override, otherwise dashboard_config().save_file.
   * @return True on success.
   */
  bool save_selected(const char* filename = nullptr) const;

  /**
   * @brief Load the selection from disk.
   * @param filename Optional override, otherwise dashboard_config().save_file.
   * @return True on success.
   */
  bool load_selected(const char* filename = nullptr);

 private:
  static void dashboard_entry(void* params);

  void handle_lcd_buttons();
  void lcd_line(int line, const std::string& text) const;
  std::string selected_line() const;
  std::string odom_line() const;
  std::string target_line() const;
  std::string battery_line() const;
  std::string temp_values_line() const;
  std::string temp_bar_line() const;
  std::string temp_bar(int percent) const;
  std::string clip(const std::string& text, std::size_t max_len = 44) const;
  const char* file_or_default(const char* filename) const;

  std::vector<AutonRoutine> routines_;
  int selected_ = 0;

  Odometry* odom_ = nullptr;
  Chassis* chassis_ = nullptr;
  pros::Controller* controller_ = nullptr;
  AutonDashboardConfig dashboard_config_;
  pros::Task* dashboard_task_ = nullptr;
  bool dashboard_running_ = false;
  std::uint32_t last_lcd_buttons_ = 0;
  const char* status_ = "Ready";
};

}  // namespace Vortex
