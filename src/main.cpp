#include "main.h"
#include "globals.h"
#include "pros/adi.hpp"
#include "robot/auton.h"
#include "pros/apix.h"
#include "display.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>

using namespace lemlib;

std::string team_number = "6079B";

void initialize() {
// Initialize the Brain screen.
     // Keep the robot stationary while this runs.
    chassis.calibrate();
    rollerMotors.set_brake_mode_all(pros::MotorBrake::brake);
    rollerMotors.brake();

    robot_ui::initialize({
        {"DO NOTHING", nullptr},

        // Replace these function names with your actual routines.
        {"LEFT SIDE", auto_left},
        {"RIGHT SIDE", auto_right},
        {"SKILLS", auto_skills},
        {"TEST", auto_test}
    });
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    rollerMotors.brake();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {

}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

ASSET(test_txt);

void autonomous() {

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    rollerMotors.brake();
    bool autonTestPaused = false;

    chassis.setPose(0, 0,0);

    bool intakeState = false;
    bool outputState = false;
    bool liftState = false;
    bool armState = false;
    bool ClawState = false;

    while (true) {
        // get joystick positions
        if (robot_ui::isAutonTestRunning()) {
            if (!autonTestPaused) {
                rollerMotors.brake();
                autonTestPaused = true;
            }
            pros::delay(20);
            continue;
        }
        autonTestPaused = false;
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);

        // The optical sensor has been removed. Keep manual roller control.
        // Down is now unassigned; there is no automatic color/drive-hold mode.
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            rollerMotors.move(-127);
        } else {
            rollerMotors.brake();
        }
        // delay to save resources
        pros::delay(10); //this is super important, never delete this line
       
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            chainLift.move(127);       // Hold R2: intake forward
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            chainLift.move(-127);      // Hold L2: intake reverse
        }
        else {
            chainLift.move(0);         // Release: stop powering the intake
        } 
        chainLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

       
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        liftMotors.move(127);// Move up
        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        liftMotors.move(-100);// Move down
         } 
        else {
        liftMotors.brake();// Hold current position
        }
        liftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            if (ClawState == false) {
                Claw.set_value(true);
                ClawState = true;
            } else {
                Claw.set_value(false);
                ClawState = false;
            }
        }

    pros::delay(20);
    }
} 