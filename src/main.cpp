#include "main.h"
#include "globals.h"
#include "claw_toggle.hpp"
#include "robot/auton.h"
#include "pros/apix.h"
#include "display.hpp"
#include "lift_calibration.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
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

    clawMotors.set_brake_mode_all(pros::MotorBrake::brake);
    clawMotors.brake();
    liftMotors.set_brake_mode_all(pros::MotorBrake::hold);

    robot_ui::initialize({
        {"DO NOTHING", nullptr},

        // Replace these function names with your actual routines.
        {"LEFT SIDE", auto_left},
        {"RIGHT SIDE", auto_right},
        {"SKILLS", auto_skills},
        {"TEST", auto_test},
        {"DISTANCE TEST", auto_distance_test}
    });
}
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    liftMotors.brake();
    rollerMotors.brake();
    clawMotors.brake();
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
    clawMotors.brake();
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
    clawMotors.brake();
    ClawToggle clawToggle;
    bool clawControlsArmed = false;
    bool clawMotorErrorReported = false;
    bool autonTestPaused = false;
    std::optional<double> liftTarget;
    std::optional<std::uint32_t> liftBrakedAt;
    bool liftAscending = false;
    std::uint32_t liftMoveStarted = 0;
    std::uint32_t liftLastProgress = 0;
    double liftProgressHeight = 0.0;
    bool liftPresetButtonsReady = false;

    constexpr double LIFT_TOLERANCE_INCHES = 0.4;
    constexpr std::uint32_t LIFT_MOVE_TIMEOUT_MS = 5000;
    constexpr std::uint32_t LIFT_BOOST_INTERVAL_MS = 300;
    constexpr double LIFT_PROGRESS_INCHES = 0.10;
    constexpr int LIFT_MAX_UP_POWER = 127;
    constexpr int LIFT_MAX_DOWN_POWER = 75;
    constexpr double LIFT_SCORING_MARGIN_INCHES = 0.5;
    constexpr std::uint32_t LIFT_SETTLE_MS = 150;
    constexpr int LIFT_MAX_STALL_BOOST = 55;
    constexpr double LIFT_POWER_PER_INCH = 36.0;
    constexpr double AUTO_LIFT_CEILING = std::min(
        lift_calibration::MAX_CALIBRATED_HEIGHT_INCHES,
        lift_calibration::MAX_SAFE_HEIGHT_INCHES);

    chassis.setPose(0, 0,0);

    while (true) {
        const auto connection = controller.is_connected();
        const auto forward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        const auto reverse = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        // PROS passes through the VEX connection status, including tethered mode.
        const bool inputsValid = connection > 0 && connection != PROS_ERR &&
                                 forward >= 0 && forward != PROS_ERR &&
                                 reverse >= 0 && reverse != PROS_ERR;
        const bool forwardHeld = inputsValid && forward > 0;
        const bool reverseHeld = inputsValid && reverse > 0;
        if (!inputsValid) clawControlsArmed = false;
        else if (!forwardHeld && !reverseHeld) clawControlsArmed = true;
        const int clawPower = clawToggle.update(forwardHeld, reverseHeld);
        // get joystick positions
        if (robot_ui::isAutonTestRunning()) {
            clawToggle.stop();
            liftTarget.reset();
            liftPresetButtonsReady = false;
            if (!autonTestPaused) {
                rollerMotors.brake();
                clawMotors.brake();
                autonTestPaused = true;
            }
            pros::delay(20);
            continue;
        }
        autonTestPaused = false;
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, -rightX);

        // The optical sensor has been removed. Keep manual roller control.
        // Down is now unassigned; there is no automatic color/drive-hold mode.
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            rollerMotors.move(-127);
        } else {
            rollerMotors.brake();
        }
        // delay to save resources
        pros::delay(10); //this is super important, never delete this line

        if (!clawControlsArmed || clawPower == 0) {
            clawToggle.stop();
            clawMotors.brake();
            clawMotorErrorReported = false;
        } else {
            const bool motorError = clawMotors.move(clawPower) == PROS_ERR;
            if (motorError && !clawMotorErrorReported) {
                std::puts("Claw motor command failed. Check Smart Port 12.");
            }
            clawMotorErrorReported = motorError;
        }


        const auto manualUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        const auto manualDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        const auto stepUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        const auto stepDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        const bool liftInputsValid = connection > 0 && connection != PROS_ERR &&
            (manualUp == 0 || manualUp == 1) && (manualDown == 0 || manualDown == 1) &&
            (stepUp == 0 || stepUp == 1) && (stepDown == 0 || stepDown == 1);

        if (!liftInputsValid || manualUp || manualDown) {
            liftTarget.reset();
            liftPresetButtonsReady = false;
            if (liftInputsValid && manualUp != manualDown) {
                liftMotors.move(manualUp ? 127 : -100);
            } else {
                liftMotors.brake();
            }
        } else {
            const auto height = robot_ui::readLiftHeightInches();
            const bool heightValid = height && *height >= -LIFT_TOLERANCE_INCHES &&
                                     *height <= AUTO_LIFT_CEILING;
            if (heightValid && liftPresetButtonsReady && stepUp != stepDown) {
                const double from = liftTarget.value_or(*height);
                std::optional<double> next;
                if (stepDown && from > LIFT_TOLERANCE_INCHES) next = 0.0;
                for (double preset : lift_calibration::SCORING_HEIGHTS_INCHES) {
                    if (preset > AUTO_LIFT_CEILING) break;
                    if (stepUp && preset > from + LIFT_TOLERANCE_INCHES) {
                        next = preset;
                        break;
                    }
                    if (stepDown && preset < from - LIFT_TOLERANCE_INCHES) next = preset;
                }
                if (next) {
                    liftTarget = next;
                    liftBrakedAt.reset();
                    liftAscending = *height < *next;
                    liftMoveStarted = pros::millis();
                    liftLastProgress = liftMoveStarted;
                    liftProgressHeight = *height;
                }
            }
            liftPresetButtonsReady = heightValid && !stepUp && !stepDown;

            const auto now = pros::millis();
            if (heightValid && liftTarget && now - liftMoveStarted >= LIFT_MOVE_TIMEOUT_MS) {
                std::printf("Lift preset timed out at %.2f in (target %.2f in)\n",
                            *height, *liftTarget);
                liftTarget.reset();
            }
            if (!heightValid) {
                liftTarget.reset();
            }
            if (liftTarget) {
                const bool scoring = *liftTarget > 0.0;
                const double stopHeight = scoring
                    ? std::min(*liftTarget + LIFT_SCORING_MARGIN_INCHES, AUTO_LIFT_CEILING)
                    : LIFT_TOLERANCE_INCHES;
                const bool reachedStopHeight = liftAscending ? *height >= stopHeight : *height <= stopHeight;

                if (liftBrakedAt || reachedStopHeight) {
                    liftMotors.brake();
                    if (!liftBrakedAt) liftBrakedAt = now;
                    if (now - *liftBrakedAt >= LIFT_SETTLE_MS) {
                        liftBrakedAt.reset();
                        if (scoring && *height < *liftTarget) {
                            liftAscending = true;
                            liftProgressHeight = *height;
                            liftLastProgress = now;
                        } else {
                            liftTarget.reset();
                        }
                    }
                } else {
                    const int maxPower = liftAscending ? LIFT_MAX_UP_POWER : LIFT_MAX_DOWN_POWER;
                    const double minPower = liftAscending ? 30.0 : 12.0;
                    int power = static_cast<int>(std::clamp(
                        std::abs(stopHeight - *height) * LIFT_POWER_PER_INCH,
                        minPower, static_cast<double>(maxPower)));
                    if (liftAscending) {
                        // Restore low approach power as soon as the lift moves again.
                        if (*height >= liftProgressHeight + LIFT_PROGRESS_INCHES) {
                            liftProgressHeight = *height;
                            liftLastProgress = now;
                        }
                        const auto stalledIntervals = (now - liftLastProgress) / LIFT_BOOST_INTERVAL_MS;
                        if (stalledIntervals > 0) {
                            const int boostFloor = 20 + 10 * static_cast<int>(std::min(stalledIntervals, std::uint32_t{4}));
                            power = std::max(power, std::min(boostFloor, LIFT_MAX_STALL_BOOST));
                        }
                    }
                    if (liftMotors.move(liftAscending ? power : -power) == PROS_ERR) {
                        liftTarget.reset();
                        liftMotors.brake();
                    }
                }
            } else {
                liftMotors.brake();
            }
        }

    pros::delay(20);
    }
}
