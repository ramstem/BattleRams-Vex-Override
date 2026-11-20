#pragma once

#include <cstdint>
#include <initializer_list>
#include <optional>

namespace robot_ui {

using AutonFunction = void (*)();

struct AutonOption {
    const char* name;
    AutonFunction function;
};

void initialize(
    std::initializer_list<AutonOption> autonomousRoutines
);

void runSelectedAutonomous();

const char* getSelectedAutonomousName();

// Height relative to the lift zero captured at startup.
std::optional<double> readLiftHeightInches();
// Capture the current encoder position after the lift has reached bottom.
bool zeroLiftHeight();

bool isAutonTestRunning();
void recordTurnTestTime(std::uint32_t elapsedMs);
void stopRobotAfterAutonTest();

}  // namespace robot_ui
