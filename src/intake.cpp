#include "intake.hpp"  // IWYU pragma: keep

#include "pros/rtos.hpp"  // IWYU pragma: keep

Intake intake;

void Intake::setState(IntakeState state, std::optional<int> customPower) {
  StateConfig config = getStateConfig(state);
  if (customPower.has_value()) {
    config.power = customPower.value();
  }

  if (m_initialized && state == m_lastState && config.power == m_lastPower) {
    return;
  }

  m_lastState = state;
  m_lastPower = config.power;
  m_initialized = true;

  applyHardware(config);
}

void Intake::deployScraperAfter(std::uint32_t delayMs) {
  pros::Task([delayMs]() {
    pros::delay(delayMs);
    scraper.set_value(true);
  });
}

void Intake::updateOpControl() {
  const bool btnL1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
  const bool btnL2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
  const bool btnR1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
  const bool btnR2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
  const bool btnDOWN = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);

  IntakeState nextState = IntakeState::Off;

  if (btnR1) {
    nextState = IntakeState::ScoreTop;
  } else if (btnL2) {
    nextState = IntakeState::Outtake;
  } else if (btnR2) {
    nextState = IntakeState::ScoreMid;
  } else if (btnL1) {
    nextState = IntakeState::Store;
  } else if (btnDOWN) {
    nextState = IntakeState::wings;
  }

  setState(nextState);
}

Intake::StateConfig Intake::getStateConfig(IntakeState state) const {
  switch (state) {
    case IntakeState::Off:
      return {0, true, true, true};
    case IntakeState::ScoreTop:
      return {127, true, false, true};
    case IntakeState::ScoreMid:
      return {127, false, true, true};
    case IntakeState::Store:
      return {127, true, true, true};
    case IntakeState::Outtake:
      return {-127, true, true, false};
    case IntakeState::wings:
      return {0, true, false, true};
  }
  return {0, true, true, true};
}

void Intake::applyHardware(const StateConfig& config) {
  intake_lower.move(config.power);
  intake_upper.move(config.power);
  flap.set_value(config.flapExtended);
  wings.set_value(config.wingsExtended);
  intake_piston.set_value(config.intakeExtended);
}