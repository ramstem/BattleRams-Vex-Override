#include "globals.h"
#include "display.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h" // IWYU pragma: export
#include "robot/auton.h"
#include "functions.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <optional>
#include <string>

using namespace lemlib;

// Start at (0, 0) facing +Y, with a zeroed lift and clear space for the route.
// Coordinates are inches from the starting pose. Heading increases clockwise.
//write my auton here
void auto_starter_test() {

    chassis.setPose(0, 0, 0);
    // Capture the wall with the robot stopped at its starting pose, facing +Y.
    // The left sensor face is 5 inches left of the robot center.
    const auto matchloadWallX = []() -> std::optional<double> {
        constexpr double sideSensorOffset = 5.0;
        std::array<int, 7> readingsMm{};
        for (std::size_t i = 0; i < readingsMm.size(); ++i) {
            const auto mm = distanceLeft.get();
            if (mm < 20 || mm > 2000) return std::nullopt;
            readingsMm[i] = mm;
            if (i + 1 < readingsMm.size()) pros::delay(20);
        }
        std::sort(readingsMm.begin(), readingsMm.end());
        if (readingsMm.back() - readingsMm.front() > 12.7) return std::nullopt;
        return -(sideSensorOffset + readingsMm[readingsMm.size() / 2] / 25.4);
    }();
    if (!matchloadWallX) {
        std::puts("Matchload: startup wall reading rejected. Using odometry.");
    }

    const auto returnToMatchloader = [&] {
        chassis.turnToHeading(180, 1000, {}, false);
        // At 180 degrees the same wall is on the right. Correct X before
        // the reverse approach. The wall reading does not determine Y.
        if (matchloadWallX) {
            pros::delay(100);
            if (!correctPositionFromWall(WallSensor::Right, WallAxis::X, *matchloadWallX)) {
                std::puts("Matchload: right wall correction rejected. Using odometry.");
            }
        }
        chassis.moveToPoint(0, -5, 1000, {.forwards = false, .maxSpeed = 80}, false);
    };

    chassis.moveToPoint(0, -6, 500, {.forwards=false, .minSpeed=127}, true); //back away from wall
    moveLiftToPreset(1, 500);
    chassis.turnToHeading(270, 500, {.maxSpeed = 45, .minSpeed = 23, .earlyExitRange = 7.5f}, false); //turn to goal
    chassis.moveToPoint(25,-17, 1000, {.forwards=false}, false); //drive to goal
    liftMotors.move(-127);
    pros::delay(500);
    liftMotors.brake();
    clawMotors.move(-127); //score
    pros::delay(500);
    chassis.moveToPoint(0,-21, 1000, {.forwards=true}, false); //drive away from goal
    clawMotors.move(127);
    returnToMatchloader();
    chassis.moveToPoint(0,-18, 1000, {.forwards=true},true); //back away 
    liftMotors.move(127);
    pros::delay(750);
    liftMotors.brake();
    chassis.turnToHeading(270, 1000); // turn to goal
    chassis.moveToPoint(23,-17, 1000, {.forwards=false}, false); //drive to goal
    liftMotors.move(-127); //score
    pros::delay(750);
    liftMotors.brake();
    clawMotors.move(-127); //score
    chassis.moveToPoint(0,-23, 1000, {.forwards=true}, false); //drive away from goal
    clawMotors.move(127);
    returnToMatchloader();
    chassis.moveToPoint(0,-18, 1000, {.forwards=true},true); //back away
    liftMotors.move(127);
    pros::delay(1250);
    liftMotors.brake();
    chassis.turnToHeading(270, 1000); // turn to goal
    chassis.moveToPoint(23,-17, 300, {.forwards=false}, false); //drive to goal
     chassis.moveToPoint(23,-17, 500, {.forwards=false,.minSpeed=40}, false); //drive to goal
    liftMotors.move(-127);
    pros::delay(500);
    liftMotors.brake();
    clawMotors.move(-127);
    chassis.moveToPoint(0,-21, 1000, {.forwards=true}, false); //drive away from goal
    chassis.turnToHeading(180, 1000); // turn to matchload
    clawMotors.move(127);


    pros::delay(500);

    
    
/*
    constexpr float firstY = 24.0f;
    constexpr float approachX = 12.0f;
    constexpr float turnHeading = 90.0f;
    constexpr float positionTolerance = 2.0f;
    constexpr float headingTolerance = 2.0f;

    const auto reachedPoint = [](float x, float y) {
        const auto pose = chassis.getPose();
        return std::hypot(pose.x - x, pose.y - y) <= positionTolerance;
    };

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose(0, 0, 0);

    // Blocking 24-inch drive using the tested straight-drive speed cap.
    chassis.moveToPoint(0, firstY, 4000, {.maxSpeed = 45}, false);
    pros::delay(150);
    if (!reachedPoint(0, firstY)) {
        std::puts("AUTON TEST: first drive missed its target");
        return;
    }

    // Wall checkpoint 1: after coming to rest at heading 0, the front sensor
    // can correct Y against a measured wall perpendicular to +Y.
    // correctPositionFromWall(WallSensor::Front, WallAxis::Y, measuredWallY);

    // Tested from rest for a 90-degree clockwise turn. The early exit lets
    // the chassis settle near 90 degrees before the next move.
    chassis.turnToHeading(turnHeading, 3000,
                          {.maxSpeed = 45, .minSpeed = 23, .earlyExitRange = 7.5f}, false);
    leftMotors.brake();
    rightMotors.brake();
    pros::delay(500);
    const double headingError = std::remainder(chassis.getPose().theta - turnHeading, 360.0);
    if (!(std::abs(headingError) <= headingTolerance)) {
        std::puts("AUTON TEST: turn missed its target");
        return;
    }

    // Wall checkpoint 2: at rest facing +X, the front sensor can correct X.
    // correctPositionFromWall(WallSensor::Front, WallAxis::X, measuredWallX);
    // Use measured wall coordinates in this starting-pose frame and a clear
    // view of the wall. If correction returns false, the odometry pose is kept.

    chassis.moveToPoint(approachX, firstY, 2000, {.maxSpeed = 35}, false);
    pros::delay(150);
    if (!reachedPoint(approachX, firstY)) {
        std::puts("AUTON TEST: approach missed its target");
        return;
    }

    // For an approach based on standoff, use driveToDistance() instead of the
    // last move once the front target is verified. It stops at a distance from
    // the sensor face. Use a separate wall checkpoint to update odometry.

    // Raise to preset 1 (10 inches) at rest. Check the result before adding
    // a scoring action. The lift holds at the preset when the test ends.
    if (!moveLiftToPreset(1, 3000)) {
        std::puts("AUTON TEST: lift did not reach preset 1");
        return;
    }
    std::puts("AUTON TEST: drive, turn, approach, and lift completed"); */
}

void auto_distance_test() {
    const bool reached = driveToDistance(6.0, 40, 3000);
    std::cout << "Distance test: " << (reached ? "reached target" : "stopped without reaching target") << std::endl;
}

void auto_test() {
    constexpr float targetY = 24.0f;
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, targetY, 4000, {.maxSpeed = 45}, false);
    pros::delay(750);
    const auto pose = chassis.getPose();
    std::printf("PID TEST: target Y=%.1f in, final X=%.2f in, final Y=%.2f in, heading=%.2f deg\n",
                targetY, pose.x, pose.y, pose.theta);
}

void auto_straight_test() {
    constexpr int drivePower = 40;
    constexpr int driveTimeMs = 500;
    chassis.setPose(0, 0, 0);
    leftMotors.move(drivePower);
    rightMotors.move(drivePower);
    pros::delay(driveTimeMs);
    leftMotors.brake();
    rightMotors.brake();

    pros::delay(500);
    const auto pose = chassis.getPose();
    std::printf("STRAIGHT TEST: final X=%.2f in, final Y=%.2f in, heading=%.2f deg\n",
                pose.x, pose.y, pose.theta);
}

void auto_turn_test() {
    constexpr float targetHeading = 90.0f;
    chassis.setPose(0, 0, 0);
    const auto started = pros::millis();
    // Tested 90-degree clockwise turn from rest: maxSpeed 45, minSpeed 23, earlyExitRange 7.5 degrees.
    // Three runs finished at 89.6 to 91.1 degrees in 750 to 760 ms of motion time.
    // Verify the cutoff separately for other turn angles and directions.
    chassis.turnToHeading(targetHeading, 3000,
                          {.maxSpeed = 45, .minSpeed = 23, .earlyExitRange = 7.5f}, false);
    const auto elapsedMs = pros::millis() - started;
    leftMotors.brake();
    rightMotors.brake();

    pros::delay(500);
    const auto pose = chassis.getPose();
    robot_ui::recordTurnTestTime(elapsedMs);
    std::printf("TURN TEST: target=%.1f deg, final heading=%.2f deg, motion time=%lu ms\n",
                targetHeading, pose.theta, static_cast<unsigned long>(elapsedMs));
}

void auto_skills() {
    chassis.setPose(0, 0,0);
    chassis.moveToPoint(0, 24, 3000);
    chassis.waitUntilDone();
}

 void auto_left() {   
    chassis.setPose(0, 0,0);
 }

void auto_right() {
    chassis.setPose(0,0,0); 
    chassis.moveToPoint(0, -24, 500,{.forwards = false,.minSpeed = 127});
    chassis.moveToPoint(0, 1, 1000,{.forwards = true,.minSpeed = 127});
}
