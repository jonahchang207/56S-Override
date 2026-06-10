#pragma once

#include <cstdint>
#include <optional>

#include "config.hpp"

enum class IntakeState {
    Off,
    ScoreTop,
    ScoreMid,
    Store,
    Outtake,
    wings,
};

class Intake {
   public:
    // Drive the intake to a state. Repeat calls with the same state are no-ops.
    // customPower overrides the state's default motor voltage if provided.
    void setState(IntakeState state, std::optional<int> customPower = std::nullopt);

    // Read controller buttons and apply the matching state. Call once per loop.
    void updateOpControl();

    // Deploy the scraper after the given delay (milliseconds). Non-blocking:
    // spawns a detached task so the caller (e.g. chassis motion) keeps running.
    void deployScraperAfter(std::uint32_t delayMs);

   private:
    struct StateConfig {
        int power;
        bool flapExtended;
        bool wingsExtended;
        bool intakeExtended;
    };

    StateConfig getStateConfig(IntakeState state) const;
    void applyHardware(const StateConfig& config);

    bool m_initialized = false;
    IntakeState m_lastState = IntakeState::Off;
    int m_lastPower = 0;
};

extern Intake intake;