#pragma once

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

bool isAutonTestRunning();
void stopRobotAfterAutonTest();

}  // namespace robot_ui