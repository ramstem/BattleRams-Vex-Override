#include "globals.h"
#include "lemlib/chassis/chassis.hpp"
#include "main.h" // IWYU pragma: export
#include "robot/auton.h"
#include "functions.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

using namespace lemlib;

void auto_distance_test() {
    const bool reached = driveToDistance(6.0, 40, 3000);
    std::cout << "Distance test: " << (reached ? "reached target" : "stopped without reaching target") << std::endl;
}

void auto_test() {
    const auto report = [](const char* step, double targetY, double targetHeading) {
        pros::delay(750);
        const auto pose = chassis.getPose();
        const double headingError = std::remainder(pose.theta - targetHeading, 360.0);
        std::printf("PID %s: x=%.2f y=%.2f heading=%.2f, y error=%+.2f, heading error=%+.2f\n",
                    step, pose.x, pose.y, pose.theta, pose.y - targetY, headingError);
    };
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 3000, {.maxSpeed = 70}, false);
    report("forward 24 in", 24, 0);
    chassis.moveToPoint(0, 0, 3000, {.forwards = false, .maxSpeed = 70}, false);
    report("reverse to start", 0, 0);
    chassis.turnToHeading(90, 2500, {.maxSpeed = 70}, false);
    report("clockwise to 90 deg", 0, 90);
    chassis.turnToHeading(0, 2500, {.maxSpeed = 70}, false);
    report("counterclockwise to 0 deg", 0, 0);
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
