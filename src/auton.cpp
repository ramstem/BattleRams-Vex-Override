#include "globals.h"
#include "lemlib/chassis/chassis.hpp"
#include "main.h" // IWYU pragma: export
#include "robot/auton.h"
#include "functions.h"
#include <cstdio>
#include <iostream>
#include <string>

using namespace lemlib;

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
    chassis.turnToHeading(targetHeading, 3000,
                          {.maxSpeed = 45, .minSpeed = 23, .earlyExitRange = 2.0f}, false);
    const auto elapsedMs = pros::millis() - started;
    leftMotors.brake();
    rightMotors.brake();

    pros::delay(500);
    const auto pose = chassis.getPose();
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
