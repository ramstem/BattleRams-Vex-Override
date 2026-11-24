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
    chassis.turnToHeading(180, 1000); // turn to matchload

    chassis.moveToPoint(0,-5, 1000, {.forwards=false,.maxSpeed=80}, false); //get matchload
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
    chassis.turnToHeading(180, 1000,{},true); // turn to matchload
    clawMotors.move(127);

    chassis.moveToPoint(0,-5, 1000, {.forwards=false,.maxSpeed=80}, false); //get matchload
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

    // At heading 0, use a side sensor to correct X against a known side wall.

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

    // At heading 90, use a side sensor to correct Y against a known wall.
    // Use measured wall coordinates in this starting-pose frame and a clear
    // view of the wall. If correction returns false, the odometry pose is kept.

    chassis.moveToPoint(approachX, firstY, 2000, {.maxSpeed = 35}, false);
    pros::delay(150);
    if (!reachedPoint(approachX, firstY)) {
        std::puts("AUTON TEST: approach missed its target");
        return;
    }

    // For a separate rear-first approach, driveToDistance() measures the gap
    // from the rear sensor face. Use a wall checkpoint to update odometry.

    // Raise to preset 1 (10 inches) at rest. Check the result before adding
    // a scoring action. The lift holds at the preset when the test ends.
    if (!moveLiftToPreset(1, 3000)) {
        std::puts("AUTON TEST: lift did not reach preset 1");
        return;
    }
    std::puts("AUTON TEST: drive, turn, approach, and lift completed"); */
}

// Start at the matchloader, facing away from it, with the side wall on the left.
// Both sensors need a clear view of the same fixed surfaces at the return checkpoint.
void auto_distance_test() {
    struct Gaps {
        double left;
        double rear;
    };
    const auto report = [](const char* message) {
        robot_ui::recordDistanceTestStatus(message);
        std::printf("Distance-Test: %s\n", message);
    };
    constexpr int sampleCount = 7;
    constexpr int requiredSamples = 5;
    constexpr double mmPerInch = 25.4;
    const auto stableGap = [](std::array<int, sampleCount> samples) -> std::optional<double> {
        std::sort(samples.begin(), samples.end());
        const int median = samples[sampleCount / 2];
        const auto agreesWithMedian = [median](int mm) {
            return mm >= 20 && mm <= 2000 && std::abs(mm - median) <= 0.5 * mmPerInch;
        };
        if (!agreesWithMedian(median) ||
            std::count_if(samples.begin(), samples.end(), agreesWithMedian) < requiredSamples) {
            return std::nullopt;
        }
        return median / mmPerInch;
    };
    const auto readGaps = [&](const char* leftFailure, const char* rearFailure) -> std::optional<Gaps> {
        std::array<int, sampleCount> left{};
        std::array<int, sampleCount> rear{};
        for (int i = 0; i < sampleCount; ++i) {
            left[i] = distanceLeft.get();
            rear[i] = distanceRear.get();
            if (i + 1 < sampleCount) pros::delay(20);
        }
        const auto leftGap = stableGap(left);
        const auto rearGap = stableGap(rear);
        if (!leftGap || !rearGap) {
            report(!leftGap ? leftFailure : rearFailure);
            std::printf("Distance-Test: last sample left=%d mm, rear=%d mm\n", left.back(), rear.back());
            return std::nullopt;
        }
        return Gaps{*leftGap, *rearGap};
    };

    constexpr float returnHeading = 0.0f;
    constexpr float forwardInches = 24.0f;
    constexpr float leftX = -12.0f;
    constexpr float driveSpeed = 45.0f;
    constexpr double maxCorrection = 6.0;
    constexpr double finalGapTolerance = 0.75;
    const auto alignedForReadings = [](float heading) {
        return std::abs(std::remainder(heading - returnHeading, 360.0)) <= 5.0;
    };
    const TurnToHeadingParams turnParams{.maxSpeed = 45, .minSpeed = 23, .earlyExitRange = 7.5f};

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setPose(0, 0, returnHeading);
    report("Reading starting gaps...");
    const auto start = readGaps("Start blocked: check left sensor (16)",
                                "Start blocked: check rear sensor (17)");
    if (!start) return;
    std::printf("Distance-Test: start left=%.2f in, rear=%.2f in\n", start->left, start->rear);

    report("Driving test route...");
    chassis.moveToPoint(0, forwardInches, 3000, {.maxSpeed = driveSpeed}, false);
    chassis.turnToHeading(-90, 1500, turnParams, false);
    chassis.moveToPoint(leftX, forwardInches, 2000, {.maxSpeed = driveSpeed}, false);
    pros::delay(1000);
    chassis.moveToPoint(0, forwardInches, 2000, {.forwards = false, .maxSpeed = driveSpeed}, false);
    chassis.turnToHeading(returnHeading, 1500, turnParams, false);
    pros::delay(150);

    const auto pose = chassis.getPose();
    if (!alignedForReadings(pose.theta)) {
        report("Return stopped: heading outside 5 deg");
        return;
    }
    const auto checkpoint = readGaps("Return blocked: check left sensor (16)",
                                     "Return blocked: check rear sensor (17)");
    if (!checkpoint) return;

    // Near the starting heading the mounting offsets cancel in the gap differences.
    // Larger left and rear gaps correspond to positive X and Y, respectively.
    const double x = checkpoint->left - start->left;
    const double y = checkpoint->rear - start->rear;
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || y <= 0 ||
        std::abs(x - pose.x) > maxCorrection || std::abs(y - pose.y) > maxCorrection) {
        std::printf("Distance-Test: rejected sensor pose X=%.2f Y=%.2f (odom X=%.2f Y=%.2f). Test stopped.\n",
                    x, y, pose.x, pose.y);
        report("Return stopped: sensor pose rejected");
        return;
    }
    std::printf("Distance-Test: sensor pose X=%.2f in, Y=%.2f in before return\n", x, y);
    report("Backing to starting position...");
    chassis.setPose(x, y, pose.theta);
    chassis.moveToPose(0, 0, returnHeading, 3000, {.forwards = false, .maxSpeed = 35}, false);

    pros::delay(150);
    const auto final = readGaps("Finish: left reading unavailable",
                                "Finish: rear reading unavailable");
    if (!final) return;
    const auto heading = chassis.getPose().theta;
    const double leftError = final->left - start->left;
    const double rearError = final->rear - start->rear;
    const bool matched = std::abs(leftError) <= finalGapTolerance &&
                         std::abs(rearError) <= finalGapTolerance && alignedForReadings(heading);
    report(matched ? "Starting gaps matched" : "Outside return tolerance");
    std::printf("Distance-Test: left error=%+.2f in, rear error=%+.2f in, heading=%.1f deg\n",
                leftError, rearError, heading);
}

void auto_rear_distance_test() {
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
