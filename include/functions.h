#pragma once

#include <cstdint>

// Approach a target using the FRONT sensor, holding the starting IMU heading.
// Inches are measured from the sensor face, not the bumper. Never reverses.
// Call chassis.waitUntilDone() first if a LemLib movement is running.
// Blocks until reached (true), timeout, invalid readings, or field disable (false).
// Uses drivetrain BRAKE mode and leaves it enabled after returning.
bool driveToDistance(double targetInches, int maxSpeed = 40,
                     std::uint32_t timeoutMs = 2000);

void chain_lift();
void chain_lift_reverse();
void chain_lift_brake();
void lift();
void roller();

void resetLocalization(float x, float y, float theta);
bool isHeadingNear(float headingDeg, float targetDeg, float tolDeg);
void localizeNow();
