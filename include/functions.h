#pragma once

#include <cstdint>

// Approach a target using the FRONT sensor, holding the starting IMU heading.
// Inches are measured from the sensor face, not the bumper. Never reverses.
// Call chassis.waitUntilDone() first if a LemLib movement is running.
// Blocks until reached (true), timeout, invalid readings, or field disable (false).
// Uses drivetrain BRAKE mode and leaves it enabled after returning.
bool driveToDistance(double targetInches, int maxSpeed = 40,
                     std::uint32_t timeoutMs = 2000);

enum class WallSensor { Front, Left, Right };
enum class WallAxis { X, Y };

struct WallCheckpoint {
    WallSensor sensor;
    WallAxis axis;           // Coordinate held constant along the wall.
    double wallCoordinate;  // Measured wall face, in your LemLib field frame.
    double sensorRight;     // Sensor face offset from pose origin, +right.
    double sensorForward;   // Sensor face offset from pose origin, +forward.
    double maxCorrection = 6.0;
};

// All distances are inches. Mounts must face straight front/left/right.
// Requires a stopped robot and an unobstructed view of the specified wall.
// Samples for 120 ms; rejects invalid/noisy ranges, movement, poor alignment,
// and corrections exceeding maxCorrection. No motors are commanded.
// On success, updates ONLY the wall's X or Y coordinate, preserving heading.
// On failure, returns false without changing the pose.
bool correctPositionFromWall(const WallCheckpoint& checkpoint);

void chain_lift();
void chain_lift_reverse();
void chain_lift_brake();
void lift();
void roller();

void resetLocalization(float x, float y, float theta);
bool isHeadingNear(float headingDeg, float targetDeg, float tolDeg);
void localizeNow();
