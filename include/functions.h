#pragma once

#include <cstdint>

// Back toward a target using the REAR sensor, holding the starting IMU heading.
// Inches are measured from the sensor face, not the bumper. Only drives backward.
// Call chassis.waitUntilDone() first if a LemLib movement is running.
// Blocks until reached (true), timeout, invalid readings, or field disable (false).
// Uses drivetrain BRAKE mode and leaves it enabled after returning.
bool driveToDistance(double targetInches, int maxSpeed = 40,
                     std::uint32_t timeoutMs = 2000);

enum class WallSensor { Rear, Left, Right };
enum class WallAxis { X, Y };

struct WallCheckpoint {
    WallSensor sensor;
    WallAxis axis;           // Coordinate held constant along the wall.
    double wallCoordinate;  // Measured wall face, in your LemLib field frame.
    double sensorRight;     // Sensor face offset from pose origin, +right.
    double sensorForward;   // Sensor face offset from pose origin, +forward.
    double maxCorrection = 6.0;
};

// All distances are inches. Mounts must face straight rear/left/right.
// Requires a stopped robot and an unobstructed view of the specified wall.
// Samples for 120 ms; rejects invalid/noisy ranges, movement, poor alignment,
// and corrections exceeding maxCorrection. No motors are commanded.
// On success, updates ONLY the wall's X or Y coordinate, preserving heading.
// On failure, returns false without changing the pose.
bool correctPositionFromWall(const WallCheckpoint& checkpoint);

// Convenience overload for the side sensors (5 inches outward, zero forward).
// For Rear, pass a WallCheckpoint with the measured rear sensor face offsets.
// wallCoordinate is relative to the robot's STARTING position (0,0).
bool correctPositionFromWall(WallSensor sensor, WallAxis axis,
                             double wallCoordinate, double maxCorrection = 6.0);

void lift();
// Blocking scoring move for autonomous or a Brain-screen autonomous test.
// Presets are numbered 1 through 6. Start the program with the lift zeroed.
// Returns true after settling at or above the requested height.
bool moveLiftToPreset(int preset, std::uint32_t timeoutMs = 5000);
void roller();

void resetLocalization(float x, float y, float theta);
bool isHeadingNear(float headingDeg, float targetDeg, float tolDeg);
void localizeNow();
