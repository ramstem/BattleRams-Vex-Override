#include "functions.h"
#include "globals.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace {
constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;

// Sensor FACE positions relative to the robot center, in inches.
// Cross-axis offsets are assumed zero: front centered left/right; sides
// directly beside the center. Update these if mounting measurements differ.
struct SensorMount { double right; double forward; };
constexpr SensorMount FRONT_MOUNT{0.0, 2.25};
constexpr SensorMount LEFT_MOUNT{-5.0, 0.0};
constexpr SensorMount RIGHT_MOUNT{5.0, 0.0};

bool usablePose(const lemlib::Pose& pose) {
    return std::isfinite(pose.x) && std::isfinite(pose.y) &&
           std::isfinite(pose.theta);
}

bool usableImu() {
    const double heading = imu.get_heading();
    return std::isfinite(heading) && heading >= 0 && heading < 360;
}

bool fieldAllowsCorrection() {
    return !pros::competition::is_connected() ||
           (!pros::competition::is_disabled() && pros::competition::is_autonomous());
}
} // namespace

bool correctPositionFromWall(const WallCheckpoint& c) {
    if (!std::isfinite(c.wallCoordinate) || !std::isfinite(c.sensorRight) ||
        !std::isfinite(c.sensorForward) || !std::isfinite(c.maxCorrection) ||
        c.maxCorrection <= 0 || (c.axis != WallAxis::X && c.axis != WallAxis::Y)) return false;

    pros::Distance* sensor = nullptr;
    double sensorAngle = 0;
    switch (c.sensor) {
        case WallSensor::Front: sensor = &distanceFront; break;
        case WallSensor::Left: sensor = &distanceLeft; sensorAngle = -90; break;
        case WallSensor::Right: sensor = &distanceRight; sensorAngle = 90; break;
        default: return false;
    }

    if (chassis.isInMotion() || !fieldAllowsCorrection() || !usableImu()) return false;
    const auto start = chassis.getPose();
    if (!usablePose(start)) return false;

    std::array<double, 7> ranges{};
    for (std::size_t i = 0; i < ranges.size(); ++i) {
        if (chassis.isInMotion() || !fieldAllowsCorrection()) return false;
        const auto pose = chassis.getPose();
        if (!usablePose(pose) ||
            std::hypot(pose.x - start.x, pose.y - start.y) > 0.25 ||
            std::abs(std::remainder(pose.theta - start.theta, 360.0)) > 1.0) return false;

        const auto mm = sensor->get();
        if (mm < 20 || mm > 2000) return false; // Includes 9999 and PROS_ERR.
        ranges[i] = mm / 25.4;
        if (i + 1 < ranges.size()) pros::delay(20);
    }

    if (chassis.isInMotion() || !fieldAllowsCorrection() || !usableImu()) return false;
    const auto pose = chassis.getPose();
    if (!usablePose(pose) ||
        std::hypot(pose.x - start.x, pose.y - start.y) > 0.25 ||
        std::abs(std::remainder(pose.theta - start.theta, 360.0)) > 1.0) return false;

    std::sort(ranges.begin(), ranges.end());
    if (ranges.back() - ranges.front() > 0.5) return false;

    // LemLib heading: 0 faces +Y, 90 faces +X; positive turns clockwise.
    const double theta = pose.theta * DEG_TO_RAD;
    const double ray = (pose.theta + sensorAngle) * DEG_TO_RAD;
    const bool xAxis = c.axis == WallAxis::X;
    const double component = xAxis ? std::sin(ray) : std::cos(ray);
    if (std::abs(component) < std::cos(5.0 * DEG_TO_RAD)) return false;

    // Rotate the sensor face's right/forward offset into the field frame.
    const double offset = xAxis
        ? c.sensorRight * std::cos(theta) + c.sensorForward * std::sin(theta)
        : -c.sensorRight * std::sin(theta) + c.sensorForward * std::cos(theta);
    const double oldCoordinate = xAxis ? pose.x : pose.y;
    if ((c.wallCoordinate - oldCoordinate - offset) * component <= 0) return false;

    // Wall = robot coordinate + mounting offset + measured ray projection.
    const double corrected = c.wallCoordinate - offset - ranges[3] * component;
    if (std::abs(corrected - oldCoordinate) > c.maxCorrection) return false;

    chassis.setPose(xAxis ? corrected : pose.x,
                    xAxis ? pose.y : corrected, pose.theta);
    return true;
}

bool correctPositionFromWall(WallSensor sensor, WallAxis axis,
                             double wallCoordinate, double maxCorrection) {
    SensorMount mount{};
    switch (sensor) {
        case WallSensor::Front: mount = FRONT_MOUNT; break;
        case WallSensor::Left: mount = LEFT_MOUNT; break;
        case WallSensor::Right: mount = RIGHT_MOUNT; break;
        default: return false;
    }
    return correctPositionFromWall({sensor, axis, wallCoordinate,
                                   mount.right, mount.forward, maxCorrection});
}
