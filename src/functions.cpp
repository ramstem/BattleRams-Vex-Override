#include "main.h"
#include "api.h"
#include "globals.h"
#include "display.hpp"
#include "functions.h"
#include "lift_calibration.hpp"
//#include "robot/monte.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/api.hpp"
#include "pros/apix.h"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rotation.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <iostream>

using namespace lemlib;

void lift() {
    liftMotors.move(127); // move front intake at full speed
}

bool moveLiftToPreset(int preset, std::uint32_t timeoutMs) {
    constexpr auto& heights = lift_calibration::SCORING_HEIGHTS_INCHES;
    constexpr int presetCount = sizeof(heights) / sizeof(heights[0]);
    constexpr double ceiling = std::min(lift_calibration::MAX_CALIBRATED_HEIGHT_INCHES,
                                        lift_calibration::MAX_SAFE_HEIGHT_INCHES);
    const auto canMove = [] {
        if (pros::competition::is_connected()) {
            return pros::competition::is_autonomous() && !pros::competition::is_disabled();
        }
        return robot_ui::isAutonTestRunning() ||
               (pros::competition::is_autonomous() && !pros::competition::is_disabled());
    };
    if (preset < 1 || preset > presetCount || timeoutMs == 0 || !canMove()) return false;
    const double target = heights[preset - 1];
    if (target > ceiling) return false;
    const double stopHeight = std::min(target + 0.5, ceiling);

    std::optional<bool> ascending;
    std::optional<std::uint32_t> brakedAt;
    const auto started = pros::millis();
    auto lastProgress = started;
    double progressHeight = 0.0;
    while (canMove() && pros::millis() - started < timeoutMs) {
        const auto height = robot_ui::readLiftHeightInches();
        if (!height || !std::isfinite(*height) || *height < -0.4 || *height > ceiling) break;
        const auto now = pros::millis();
        if (!ascending) {
            ascending = *height < target;
            progressHeight = *height;
            lastProgress = now;
        }
        const bool reached = *ascending ? *height >= stopHeight : *height <= stopHeight;
        if (brakedAt || reached) {
            if (liftMotors.brake() == PROS_ERR) break;
            if (!brakedAt) brakedAt = now;
            if (now - *brakedAt >= 150) {
                if (*height >= target) return true;
                ascending = true;
                brakedAt.reset();
                progressHeight = *height;
                lastProgress = now;
            }
        } else {
            const double minPower = *ascending ? 30.0 : 12.0;
            const double maxPower = *ascending ? 127.0 : 75.0;
            int power = static_cast<int>(std::clamp(
                std::abs(stopHeight - *height) * 36.0, minPower, maxPower));
            if (*ascending) {
                if (*height >= progressHeight + 0.10) {
                    progressHeight = *height;
                    lastProgress = now;
                }
                const auto intervals = (now - lastProgress) / 300;
                if (intervals > 0) {
                    const int boost = 20 + 10 * static_cast<int>(std::min(intervals, std::uint32_t{4}));
                    power = std::max(power, std::min(boost, 55));
                }
            }
            if (liftMotors.move(*ascending ? power : -power) == PROS_ERR) break;
        }
        pros::delay(20);
    }
    liftMotors.brake();
    return false;
}

void roller() {
    rollerMotors.move(127); // move front intake at full speed
}
