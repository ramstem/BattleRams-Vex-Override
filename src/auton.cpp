#include "globals.h"
#include "display.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h" // IWYU pragma: export
#include "robot/auton.h"
#include "functions.h"
#include <cmath>
#include <cstdio>
#include <iostream>
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

void left_auton() {
    int i = 0;
    chassis.setPose(0, 2.5, 0);
    chassis.moveToPoint(0,8.5, 500,{.forwards=true, .maxSpeed = 25},false); //back up 
    chassis.moveToPoint(0, 0, 1500,{.forwards=false, .maxSpeed = 25},false); //go into matchload
    clawMotors.move(127); //pickup matchload
    pros::delay(500);
    for (i = 1; i < 3; i++) {
        chassis.moveToPoint(0, 8.5, 1000,{.forwards=true},true); // drive away from matchload
        liftMotors.move(127);
        pros::delay(500*i);
        liftMotors.brake();
        chassis.turnToHeading(-90, 1000,{}, false); // turn to goal
        chassis.moveToPoint(26,9, 1200,{.forwards=false},false); //drive to goal
        liftMotors.move(-127);
        pros::delay(350*i);
        liftMotors.brake();
        clawMotors.move(-127); //score
        chassis.moveToPoint(0,9, 1200,{.forwards=true},true); // drive away from goal
        liftMotors.move(-127);
        pros::delay(150*i);
        liftMotors.brake();
        clawMotors.move(127);
        chassis.turnToHeading(0, 1000,{}, false); // turn to matchload
        chassis.moveToPoint(0, 0, 1500,{.forwards=false},false); // go into matchload
        pros::delay(500);
        chassis.setPose(0,0,0); //RESET
    }
    pros::delay(500);   
}

void right_auton() {
    int i = 1;
    int liftHeight = 0;
    chassis.setPose(0, 2.5, 0);
    chassis.moveToPoint(0,8.5, 500,{.forwards=true, .maxSpeed = 25},false); //back up 
    chassis.moveToPoint(0, 0, 1500,{.forwards=false, .maxSpeed = 25},false); //go into matchload
    clawMotors.move(127); //pickup matchload
    pros::delay(250);

    chassis.moveToPoint(0, 8.5, 1000,{.forwards=true},true); // drive away from matchload
    liftMotors.move(127);
    pros::delay(500*i);
    liftMotors.brake();
    chassis.turnToHeading(90, 1000,{}, false); // turn to goal
    chassis.moveToPoint(-26,9, 1200,{.forwards=false},false); //drive to goal
    liftMotors.move(-127);
    pros::delay(250*i);
    liftMotors.brake();
    clawMotors.move(-127); //score
    chassis.moveToPoint(0,9, 1200,{.forwards=true},true); // drive away from goal
    liftMotors.move(-127);
    pros::delay(250*i);
    liftMotors.brake();
    clawMotors.move(127);
    chassis.turnToHeading(0, 1000,{}, false); // turn to matchload
    chassis.moveToPoint(0, 0, 1500,{.forwards=false},false); // go into matchload
    pros::delay(500);
    chassis.setPose(0,0,0); //RESET
    i++;
    chassis.moveToPoint(0, 9, 1200,{.forwards=true},true); // drive away from matchload
    liftMotors.move(127);
    pros::delay(750);
    liftMotors.brake();
    chassis.turnToHeading(90, 1000,{}, false); // turn to goal
    chassis.moveToPoint(-26,9, 1200,{.forwards=false},false); //drive to goal
    liftMotors.move(-127);
    pros::delay(250 );
    liftMotors.brake();
    clawMotors.move(-127); //score
}

void left_Preload() {
    int i = 1;
    int liftHeight = 0;
    chassis.setPose(0, 2.5, 0);
    for (i = 1; i < 2; i++) {
    chassis.moveToPoint(0, 8.5, 1000,{.forwards=true},true); // drive away from matchload
    liftMotors.move(127);
    pros::delay(400*i);
    liftMotors.brake();
    chassis.turnToHeading(90, 1000,{}, false); // turn to goal
    chassis.moveToPoint(-26,9, 700,{.forwards=false},false); //drive to goal
    chassis.moveToPoint(-25,9, 500,{.forwards=false, .maxSpeed = 55},false); //drive to goal
    liftMotors.move(-127);
    pros::delay(150*i);
    liftMotors.brake();
    clawMotors.move(-127); //score
    chassis.moveToPoint(0,9, 1200,{.forwards=true},true); // drive away from goal
    liftMotors.move(-127);
    pros::delay(250*i);
    liftMotors.brake();
    clawMotors.move(127);
    chassis.turnToHeading(0, 1000,{}, false); // turn to matchload
    chassis.moveToPoint(0, 0, 1500,{.forwards=false},false); // go into matchload
    pros::delay(500);
    chassis.setPose(0,0,0); //RESET

    }
}