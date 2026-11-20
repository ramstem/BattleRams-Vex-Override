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
