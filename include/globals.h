#ifndef GLOBALS_H
#define GLOBALS_H

#include "api.h"
#include "lemlib/api.hpp"
#include "pros/distance.hpp"

//controller
extern pros::Controller controller;

//drivetrain
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern lemlib::Drivetrain drivetrain;

extern pros::Imu imu;

extern pros::Distance distanceFront;
extern pros::Distance distanceLeft;
extern pros::Distance distanceRight;

extern pros::Rotation liftMotorsEnc;
extern pros::Rotation horizontalEnc;
extern lemlib::TrackingWheel horizontalTrackingWheel;
extern pros::Rotation verticalEnc;
extern lemlib::TrackingWheel verticalTrackingWheel;

extern lemlib::ControllerSettings linearController;
extern lemlib::ControllerSettings angularController;

extern lemlib::OdomSensors sensors;
extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;
extern lemlib::Chassis chassis;

//motors
extern pros::MotorGroup liftMotors;
extern pros::MotorGroup rollerMotors;
extern pros::MotorGroup clawMotors;

#endif // GLOBALS_H
