#pragma once

/**
 * @file pid_tuner.hpp
 * @brief On-robot PID gain tuner driven by a V5 controller.
 *
 * PIDTuner lets you cycle through a list of named PIDGains pointers and
 * adjust their fields (kP, kI, kD, integral_zone, max_integral) with the
 * controller buttons while the robot is sitting idle. Useful for dialing
 * in motion-control gains between practice runs without recompiling.
 */

#include "api.h"
#include "chassis.hpp"

#include <vector>

namespace Vortex {

/**
 * @class PIDTuner
 * @brief Live PID gain adjustment via the V5 controller.
 *
 * Typical setup:
 * @code
 * pid_tuner.bind_chassis(&chassis);
 * Vortex::MotionConfig* m = chassis.motion_config();
 * pid_tuner.add("Drive", &m->drive);
 * pid_tuner.add("Turn",  &m->turn);
 * @endcode
 *
 * Then in opcontrol() call iterate(master) regularly. Buttons cycle
 * through entries and fields; bumpers nudge the selected field by the
 * configured increment.
 */
class PIDTuner {
 public:
  /** @brief Provide the Chassis whose PIDs are being tuned (for live updates). */
  void bind_chassis(Chassis* chassis);

  /**
   * @brief Register every loop in the bound chassis @ref MotionConfig.
   *
   * Call after bind_chassis(). Replaces manual add() calls in @c initialize().
   */
  void bind_defaults();

  /**
   * @brief Register a PIDGains struct under a display name.
   * @param name  Display name (must outlive the tuner).
   * @param gains Pointer to the live gains struct.
   */
  void add(const char* name, PIDGains* gains);

  /** @brief Remove every registered entry. */
  void clear();

  /** @brief Enable controller-driven tuning. */
  void enable();

  /** @brief Disable controller-driven tuning. */
  void disable();

  /** @brief Toggle enable() / disable(). */
  void toggle();

  /** @brief True iff tuning is enabled. */
  bool enabled() const;

  /** @brief When true, print every change to stdout for the terminal logger. */
  void set_print_terminal(bool enabled);

  /** @brief Whether terminal logging is currently on. */
  bool print_terminal_enabled() const;

  /**
   * @brief Configure how much each bumper press changes each field.
   * @param p      kP increment.
   * @param i      kI increment.
   * @param d      kD increment.
   * @param zone   integral_zone increment.
   * @param max_i  max_integral increment.
   */
  void set_increments(double p, double i, double d, double zone,
                      double max_i);

  /**
   * @brief Poll the controller and apply input. Call from opcontrol().
   * @param controller Source of button events.
   */
  void iterate(pros::Controller& controller);

  /** @brief Print every entry and its gains to stdout. */
  void print() const;

 private:
  struct Entry {
    const char* name = "";
    PIDGains* gains = nullptr;
  };

  void adjust(double amount);
  void mark_changed();
  const char* field_name() const;
  double field_value(const PIDGains& gains) const;

  std::vector<Entry> entries_;
  Chassis* chassis_ = nullptr;
  bool enabled_ = false;
  bool print_terminal_ = true;
  int selected_entry_ = 0;
  int selected_field_ = 0;
  double inc_p_ = 1.0;
  double inc_i_ = 0.001;
  double inc_d_ = 5.0;
  double inc_zone_ = 1.0;
  double inc_max_i_ = 50.0;
};

}  // namespace Vortex
