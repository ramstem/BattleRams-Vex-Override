#include "functions.h"
#include "globals.h"
#include <algorithm>
#include <cmath>

bool driveToDistance(double targetInches, int maxSpeed, std::uint32_t timeoutMs) {
    // Do not compete with an active LemLib controller for the motors.
    if (chassis.isInMotion()) return false;

    // Match the Brain test's drivetrain braking; leave brake mode enabled.
    leftMotors.set_brake_mode_all(pros::MotorBrake::brake);
    rightMotors.set_brake_mode_all(pros::MotorBrake::brake);
    const auto stop = []() {
        leftMotors.move(0);
        rightMotors.move(0);
        leftMotors.brake();
        rightMotors.brake();
    };
    stop();
    if (!std::isfinite(targetInches) || targetInches < 1.0 ||
        targetInches * 25.4 >= 9999 || maxSpeed < 1 || maxSpeed > 127 ||
        timeoutMs == 0) return false;

    const double heading = imu.get_heading();
    if (!std::isfinite(heading) || heading < 0 || heading >= 360) return false;

    const auto started = pros::millis();
    unsigned invalidMs = 0;
    std::uint32_t settledSince = 0;
    bool settling = false;
    while (pros::millis() - started < timeoutMs) {
        // Also permits the disconnected Brain-screen autonomous test.
        if (pros::competition::is_connected() &&
            (pros::competition::is_disabled() ||
             !pros::competition::is_autonomous())) break;

        const auto mm = distanceFront.get();
        const double currentHeading = imu.get_heading();
        if (!std::isfinite(currentHeading) || currentHeading < 0 ||
            currentHeading >= 360) break;

        // 9999 is "no object"; PROS_ERR is outside this valid interval too.
        if (mm <= 0 || mm >= 9999) {
            stop();
            settling = false;
            invalidMs += 20;
            if (invalidMs >= 200) break;
        } else {
            invalidMs = 0;
            const double error = mm / 25.4 - targetInches;
            if (error <= 0.25) {
                // Already closer than the target also counts as reached.
                stop();
                if (!settling) {
                    settling = true;
                    settledSince = pros::millis();
                }
                if (pros::millis() - settledSince >= 100) return true;
            } else {
                settling = false;
                const double speed = std::clamp(error * 6.0,
                    static_cast<double>(std::min(12, maxSpeed)),
                    static_cast<double>(maxSpeed));
                const double headingError = std::remainder(heading - currentHeading, 360.0);
                const double turnLimit = std::min(15.0, speed);
                const double turn = std::clamp(headingError * 2.0, -turnLimit, turnLimit);
                leftMotors.move(static_cast<int>(std::clamp(speed + turn, 0.0, static_cast<double>(maxSpeed))));
                rightMotors.move(static_cast<int>(std::clamp(speed - turn, 0.0, static_cast<double>(maxSpeed))));
            }
        }
        pros::delay(20);
    }
    stop();
    return false;
}
