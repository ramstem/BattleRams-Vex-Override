#include "display.hpp"

#include "globals.h"
#include "main.h"
#include "liblvgl/lvgl.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <string>
#include <vector>

namespace robot_ui {
namespace {

// -----------------------------------------------------------------------------
// Expected globals from globals.hpp
// -----------------------------------------------------------------------------
// lemlib::Chassis chassis;
// pros::Imu imu;
// lemlib::TrackingWheel verticalTrackingWheel;
// lemlib::TrackingWheel horizontalTrackingWheel;
// pros::MotorGroup leftMotors;
// pros::MotorGroup rightMotors;
// pros::MotorGroup intakeMotors;
// pros::MotorGroup liftMotors;
//
// Rename the references in this file if your objects use different names.

// -----------------------------------------------------------------------------
// Display constants
// -----------------------------------------------------------------------------

constexpr std::uint32_t COLOR_BACKGROUND = 0x0D1117;
constexpr std::uint32_t COLOR_CARD = 0x161B22;
constexpr std::uint32_t COLOR_BORDER = 0x30363D;
constexpr std::uint32_t COLOR_TEXT = 0xF0F3F6;
constexpr std::uint32_t COLOR_MUTED = 0x8B949E;
constexpr std::uint32_t COLOR_ACCENT = 0xF2C94C;
constexpr std::uint32_t COLOR_NORMAL = 0x2ECC71;
constexpr std::uint32_t COLOR_WARNING = 0xF39C12;
constexpr std::uint32_t COLOR_HOT = 0xE74C3C;
constexpr std::uint32_t COLOR_UNKNOWN = 0x8B949E;

constexpr std::size_t MAX_AUTON_BUTTONS = 6;
constexpr std::uint32_t DISPLAY_REFRESH_MS = 250;

// -----------------------------------------------------------------------------
// Autonomous state
// -----------------------------------------------------------------------------

std::vector<AutonOption> autonOptions;
std::atomic<std::size_t> selectedAutonIndex{0};
std::atomic<bool> autonTestRunning{false};

// The LVGL callback sets this immediately before starting the PROS task.
// Only one test can run at a time, so no second writer is allowed until the
// task clears autonTestRunning.
AutonFunction autonTestRoutine = nullptr;

std::vector<lv_obj_t*> autonButtons;
std::vector<lv_obj_t*> autonButtonLabels;
std::array<std::size_t, MAX_AUTON_BUTTONS> autonButtonIndices{};

// -----------------------------------------------------------------------------
// LVGL objects
// -----------------------------------------------------------------------------

lv_obj_t* dashboardScreen = nullptr;
lv_obj_t* autonScreen = nullptr;
lv_obj_t* distanceScreen = nullptr;
std::array<lv_obj_t*, 3> distanceLabels{};

lv_obj_t* statusLabel = nullptr;
lv_obj_t* batteryLabel = nullptr;
lv_obj_t* poseLabel = nullptr;
lv_obj_t* headingLabel = nullptr;
lv_obj_t* trackingLabel = nullptr;

lv_obj_t* leftDriveTempLabel = nullptr;
lv_obj_t* rightDriveTempLabel = nullptr;
lv_obj_t* intakeTempLabel = nullptr;
lv_obj_t* liftTempLabel = nullptr;

lv_obj_t* selectedAutonLabel = nullptr;
lv_obj_t* selectorMessageLabel = nullptr;
lv_obj_t* runAutonButton = nullptr;
lv_obj_t* runAutonButtonLabel = nullptr;

// -----------------------------------------------------------------------------
// General UI helpers
// -----------------------------------------------------------------------------

void styleScreen(lv_obj_t* screen) {
    lv_obj_set_style_bg_color(
        screen,
        lv_color_hex(COLOR_BACKGROUND),
        LV_PART_MAIN
    );
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
}

lv_obj_t* createCard(
    lv_obj_t* parent,
    int x,
    int y,
    int width,
    int height
) {
    lv_obj_t* card = lv_obj_create(parent);

    lv_obj_set_pos(card, x, y);
    lv_obj_set_size(card, width, height);
    lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_style_bg_color(card, lv_color_hex(COLOR_CARD), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(card, lv_color_hex(COLOR_BORDER), LV_PART_MAIN);
    lv_obj_set_style_border_width(card, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(card, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_all(card, 0, LV_PART_MAIN);

    return card;
}

lv_obj_t* createLabel(
    lv_obj_t* parent,
    const char* text,
    int x,
    int y,
    int width,
    std::uint32_t color = COLOR_TEXT
) {
    lv_obj_t* label = lv_label_create(parent);

    lv_obj_set_pos(label, x, y);
    lv_obj_set_width(label, width);
    lv_label_set_text(label, text);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_color(label, lv_color_hex(color), LV_PART_MAIN);

    return label;
}

void styleButton(lv_obj_t* button, std::uint32_t backgroundColor) {
    lv_obj_set_style_bg_color(button, lv_color_hex(backgroundColor), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(button, lv_color_hex(COLOR_BORDER), LV_PART_MAIN);
    lv_obj_set_style_border_width(button, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(button, 7, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(button, 0, LV_PART_MAIN);
}

// -----------------------------------------------------------------------------
// Temperature helpers
// -----------------------------------------------------------------------------

bool isValidTemperature(double temperature) {
    return std::isfinite(temperature) &&
           temperature >= 0.0 &&
           temperature < 200.0;
}

double getMaximumTemperature(const std::vector<double>& temperatures) {
    double maximum = -1.0;

    for (const double temperature : temperatures) {
        if (isValidTemperature(temperature)) {
            maximum = std::max(maximum, temperature);
        }
    }

    return maximum;
}

lv_color_t getTemperatureColor(double temperature) {
    if (temperature < 0.0) {
        return lv_color_hex(COLOR_UNKNOWN);
    }

    if (temperature >= 55.0) {
        return lv_color_hex(COLOR_HOT);
    }

    if (temperature >= 45.0) {
        return lv_color_hex(COLOR_WARNING);
    }

    return lv_color_hex(COLOR_NORMAL);
}

std::string createTemperatureText(
    const char* name,
    const std::vector<double>& temperatures
) {
    std::string text = name;
    text += ": ";

    bool foundTemperature = false;

    for (const double temperature : temperatures) {
        if (!isValidTemperature(temperature)) {
            continue;
        }

        if (foundTemperature) {
            text += " / ";
        }

        char value[16];
        std::snprintf(value, sizeof(value), "%.0f C", temperature);
        text += value;
        foundTemperature = true;
    }

    if (!foundTemperature) {
        text += "--";
    }

    return text;
}

void updateTemperatureLabel(
    lv_obj_t* label,
    const char* name,
    const std::vector<double>& temperatures
) {
    if (label == nullptr) {
        return;
    }

    const std::string text = createTemperatureText(name, temperatures);
    const double maximum = getMaximumTemperature(temperatures);

    lv_label_set_text(label, text.c_str());
    lv_obj_set_style_text_color(label, getTemperatureColor(maximum), LV_PART_MAIN);
}

// -----------------------------------------------------------------------------
// Autonomous helpers
// -----------------------------------------------------------------------------

std::size_t getSafeSelectedAutonIndex() {
    if (autonOptions.empty()) {
        return 0;
    }

    std::size_t index = selectedAutonIndex.load();

    if (index >= autonOptions.size()) {
        index = 0;
        selectedAutonIndex.store(0);
    }

    return index;
}

AutonFunction getSelectedAutonFunction() {
    if (autonOptions.empty()) {
        return nullptr;
    }

    return autonOptions[getSafeSelectedAutonIndex()].function;
}

bool isAutonSelectionAllowed() {
    return !autonTestRunning.load() &&
           (
               !pros::competition::is_connected() ||
               pros::competition::is_disabled()
           );
}

bool isAutonTestAllowed() {
    return !pros::competition::is_connected() &&
           !autonTestRunning.load() &&
           getSelectedAutonFunction() != nullptr;
}

void stopRobotAfterAutonTest() {
    // Stop all active and queued LemLib movements.
    chassis.cancelAllMotions();

    // Use brake rather than hold on the drivetrain.
    leftMotors.set_brake_mode_all(pros::MotorBrake::brake);
    rightMotors.set_brake_mode_all(pros::MotorBrake::brake);

    leftMotors.brake();
    rightMotors.brake();

    // Stop the mechanisms using their existing brake modes.
    chainLift.move(0);
    liftMotors.move(0);
    rollerMotors.brake();
}

void updateSelectedAutonDisplay() {
    if (selectedAutonLabel == nullptr) {
        return;
    }

    if (autonOptions.empty()) {
        lv_label_set_text(selectedAutonLabel, "AUTON: NONE");
        return;
    }

    const char* name = autonOptions[getSafeSelectedAutonIndex()].name;

    char text[96];
    std::snprintf(text, sizeof(text), "AUTON: %s", name);
    lv_label_set_text(selectedAutonLabel, text);
}

void updateRunAutonButtonStyles() {
    if (runAutonButton == nullptr || runAutonButtonLabel == nullptr) {
        return;
    }

    const bool running = autonTestRunning.load();
    const bool fieldConnected = pros::competition::is_connected();
    const bool allowed = isAutonTestAllowed();

    char buttonText[112];

    if (running) {
        std::snprintf(buttonText, sizeof(buttonText), "AUTON TEST RUNNING...");
    } else if (fieldConnected) {
        std::snprintf(buttonText, sizeof(buttonText), "FIELD CONNECTED - TEST LOCKED");
    } else if (getSelectedAutonFunction() == nullptr) {
        std::snprintf(buttonText, sizeof(buttonText), "SELECT A TESTABLE AUTON ROUTINE");
    } else {
        std::snprintf(
            buttonText,
            sizeof(buttonText),
            "HOLD TO RUN: %s",
            autonOptions[getSafeSelectedAutonIndex()].name
        );
    }

    lv_label_set_text(runAutonButtonLabel, buttonText);

    const std::uint32_t backgroundColor = running
        ? COLOR_WARNING
        : allowed
            ? COLOR_ACCENT
            : COLOR_BORDER;

    lv_obj_set_style_bg_color(
        runAutonButton,
        lv_color_hex(backgroundColor),
        LV_PART_MAIN
    );

    lv_obj_set_style_text_color(
        runAutonButtonLabel,
        lv_color_hex((allowed || running) ? COLOR_BACKGROUND : COLOR_MUTED),
        LV_PART_MAIN
    );

    lv_obj_set_style_opa(
        runAutonButton,
        (allowed || running) ? LV_OPA_COVER : LV_OPA_60,
        LV_PART_MAIN
    );
}

void updateAutonButtonStyles() {
    const std::size_t selected = getSafeSelectedAutonIndex();
    const bool selectionAllowed = isAutonSelectionAllowed();

    for (std::size_t i = 0; i < autonButtons.size(); ++i) {
        const bool isSelected = i == selected;

        lv_obj_set_style_bg_color(
            autonButtons[i],
            lv_color_hex(isSelected ? COLOR_ACCENT : COLOR_CARD),
            LV_PART_MAIN
        );

        lv_obj_set_style_border_color(
            autonButtons[i],
            lv_color_hex(isSelected ? COLOR_ACCENT : COLOR_BORDER),
            LV_PART_MAIN
        );

        lv_obj_set_style_text_color(
            autonButtonLabels[i],
            lv_color_hex(isSelected ? COLOR_BACKGROUND : COLOR_TEXT),
            LV_PART_MAIN
        );

        lv_obj_set_style_opa(
            autonButtons[i],
            selectionAllowed ? LV_OPA_COVER : LV_OPA_60,
            LV_PART_MAIN
        );
    }

    if (selectorMessageLabel != nullptr) {
        if (autonTestRunning.load()) {
            lv_label_set_text(selectorMessageLabel, "Autonomous test is running");
        } else if (selectionAllowed) {
            lv_label_set_text(selectorMessageLabel, "Tap a routine to select it");
        } else {
            lv_label_set_text(selectorMessageLabel, "Selection locked while robot is enabled");
        }
    }

    updateRunAutonButtonStyles();
}

// -----------------------------------------------------------------------------
// Autonomous test task and LVGL callbacks
// -----------------------------------------------------------------------------

void autonTestTask() {
    const AutonFunction routine = autonTestRoutine;

    stopRobotAfterAutonTest();

    // Three-second safety delay before starting.
    pros::delay(3000);

    if (routine != nullptr) {
        routine();
    }

    stopRobotAfterAutonTest();

    // Keep opcontrol paused while the drivetrain settles.
    pros::delay(500);

    autonTestRoutine = nullptr;
    autonTestRunning.store(false);
}

void selectAutonEvent(lv_event_t* event) {
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    if (!isAutonSelectionAllowed()) {
        updateAutonButtonStyles();
        return;
    }

    const auto* indexPointer = static_cast<const std::size_t*>(
        lv_event_get_user_data(event)
    );

    if (indexPointer == nullptr || *indexPointer >= autonOptions.size()) {
        return;
    }

    selectedAutonIndex.store(*indexPointer);
    updateSelectedAutonDisplay();
    updateAutonButtonStyles();
}

void runAutonTestEvent(lv_event_t* event) {
    if (lv_event_get_code(event) != LV_EVENT_LONG_PRESSED) {
        return;
    }

    if (!isAutonTestAllowed()) {
        updateAutonButtonStyles();
        return;
    }

    const AutonFunction selectedRoutine = getSelectedAutonFunction();

    if (selectedRoutine == nullptr) {
        return;
    }

    bool expected = false;

    if (!autonTestRunning.compare_exchange_strong(expected, true)) {
        return;
    }

    autonTestRoutine = selectedRoutine;
    updateAutonButtonStyles();

    const auto task = pros::Task::create(
        autonTestTask,
        "Auton Test"
    );

    if (task == nullptr) {
        autonTestRoutine = nullptr;
        autonTestRunning.store(false);
        updateAutonButtonStyles();
    }
}

void showAutonSelectorEvent(lv_event_t* event) {
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    updateAutonButtonStyles();

    lv_screen_load_anim(
        autonScreen,
        LV_SCR_LOAD_ANIM_MOVE_LEFT,
        200,
        0,
        false
    );
}

void showDashboardEvent(lv_event_t* event) {
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) {
        return;
    }

    lv_screen_load_anim(
        dashboardScreen,
        LV_SCR_LOAD_ANIM_MOVE_RIGHT,
        200,
        0,
        false
    );
}

void showDistanceEvent(lv_event_t* event) {
    if (lv_event_get_code(event) == LV_EVENT_CLICKED) {
        lv_screen_load(distanceScreen);
    }
}

// -----------------------------------------------------------------------------
// Dashboard updates
// -----------------------------------------------------------------------------

void updateCompetitionStatus() {
    if (statusLabel == nullptr) {
        return;
    }

    const char* mode = "DRIVER";

    if (autonTestRunning.load()) {
        mode = "AUTON TEST";
    } else if (pros::competition::is_disabled()) {
        mode = "DISABLED";
    } else if (pros::competition::is_autonomous()) {
        mode = "AUTONOMOUS";
    }

    char text[40];

    std::snprintf(
        text,
        sizeof(text),
        "%s%s",
        mode,
        pros::competition::is_connected() ? " | FIELD" : ""
    );

    lv_label_set_text(statusLabel, text);
}

void updateBatteryStatus() {
    if (batteryLabel == nullptr) {
        return;
    }

    const double batteryCapacity = pros::battery::get_capacity();
    const double batteryVoltage = pros::battery::get_voltage() / 1000.0;

    char text[40];
    std::snprintf(
        text,
        sizeof(text),
        "%.0f%% | %.2f V",
        batteryCapacity,
        batteryVoltage
    );

    lv_label_set_text(batteryLabel, text);
}

void updateOdometryStatus() {
    if (
        poseLabel == nullptr ||
        headingLabel == nullptr ||
        trackingLabel == nullptr
    ) {
        return;
    }

    const lemlib::Pose pose = chassis.getPose();
    const double verticalDistance = verticalTrackingWheel.getDistanceTraveled();
    const double horizontalDistance = horizontalTrackingWheel.getDistanceTraveled();
    errno = 0;
    const double imuHeading = imu.get_heading();
    // Capture immediately: subsequent device calls can overwrite errno.
    const int imuError = errno;

    char poseText[80];
    char headingText[96];
    char imuText[32];
    char trackingText[104];

    // Use snprintf rather than lv_label_set_text_fmt for floating-point values.
    std::snprintf(
        poseText,
        sizeof(poseText),
        "X: %7.2f in     Y: %7.2f in",
        pose.x,
        pose.y
    );

    if (std::isfinite(imuHeading) && imuHeading >= 0.0 && imuHeading < 360.0) {
        std::snprintf(imuText, sizeof(imuText), "%6.1f deg", imuHeading);
    } else if (imuError == EAGAIN) {
        std::snprintf(imuText, sizeof(imuText), "CALIBRATING");
    } else {
        // PROS_ERR_F is infinity. Show the actual read failure rather than
        // presenting it as an angle, or hiding it behind a stale value.
        std::snprintf(imuText, sizeof(imuText), "ERR %d", imuError);
    }

    std::snprintf(
        headingText,
        sizeof(headingText),
        "LemLib: %6.1f deg   IMU: %s",
        pose.theta,
        imuText
    );

    std::snprintf(
        trackingText,
        sizeof(trackingText),
        "Vertical: %7.2f   Horizontal: %7.2f",
        verticalDistance,
        horizontalDistance
    );

    lv_label_set_text(poseLabel, poseText);
    lv_label_set_text(headingLabel, headingText);
    lv_label_set_text(trackingLabel, trackingText);
}

void updateMotorTemperatures() {
    updateTemperatureLabel(
        leftDriveTempLabel,
        "LEFT DRIVE",
        leftMotors.get_temperature_all()
    );

    updateTemperatureLabel(
        rightDriveTempLabel,
        "RIGHT DRIVE",
        rightMotors.get_temperature_all()
    );

    updateTemperatureLabel(
        intakeTempLabel,
        "CHAIN LIFT",
        chainLift.get_temperature_all()
    );

    updateTemperatureLabel(
        liftTempLabel,
        "LIFT",
        liftMotors.get_temperature_all()
    );
}

void updateDistanceSensors() {
    const std::array<pros::Distance*, 3> sensors{
        &distanceFront, &distanceLeft, &distanceRight
    };
    for (std::size_t i = 0; i < sensors.size(); ++i) {
        if (distanceLabels[i] == nullptr) continue;
        const auto mm = sensors[i]->get();
        char text[64];
        if (mm == PROS_ERR) {
            std::snprintf(text, sizeof(text), "SENSOR ERROR");
        } else if (mm < 20 || mm > 2000) {
            // Includes the no-object/out-of-range reading. Never show it as
            // a usable distance; these readings must not drive an approach.
            std::snprintf(text, sizeof(text), "NO USABLE RANGE");
        } else {
            std::snprintf(text, sizeof(text), "%ld mm / %.1f in",
                          static_cast<long>(mm), mm / 25.4);
        }
        lv_label_set_text(distanceLabels[i], text);
    }
}

void dashboardUpdateTimer(lv_timer_t*) {
    updateCompetitionStatus();
    updateBatteryStatus();
    updateOdometryStatus();
    updateMotorTemperatures();
    updateDistanceSensors();
    updateSelectedAutonDisplay();
    updateAutonButtonStyles();
}

// -----------------------------------------------------------------------------
// Screen construction
// -----------------------------------------------------------------------------

void createDashboardScreen() {
    dashboardScreen = lv_obj_create(nullptr);
    styleScreen(dashboardScreen);

    lv_obj_t* header = createCard(dashboardScreen, 5, 4, 470, 34);

    createLabel(header, "BATTLE RAMS | OVERRIDE", 10, 8, 190);

    statusLabel = createLabel(header, "DISABLED", 195, 8, 145, COLOR_ACCENT);
    lv_obj_set_style_text_align(statusLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    batteryLabel = createLabel(header, "--% | -- V", 340, 8, 120, COLOR_MUTED);
    lv_obj_set_style_text_align(batteryLabel, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);

    lv_obj_t* odometryCard = createCard(dashboardScreen, 5, 43, 265, 137);

    createLabel(
        odometryCard,
        "ODOMETRY AND TRACKING",
        10,
        8,
        245,
        COLOR_ACCENT
    );

    poseLabel = createLabel(odometryCard, "X: --   Y: --", 10, 38, 245);
    headingLabel = createLabel(
        odometryCard,
        "LemLib: --   IMU: --",
        10,
        69,
        245
    );
    trackingLabel = createLabel(
        odometryCard,
        "Vertical: --   Horizontal: --",
        10,
        100,
        245,
        COLOR_MUTED
    );

    lv_obj_t* temperatureCard = createCard(dashboardScreen, 275, 43, 200, 137);

    createLabel(
        temperatureCard,
        "MOTOR TEMPERATURES",
        10,
        8,
        180,
        COLOR_ACCENT
    );

    leftDriveTempLabel = createLabel(
        temperatureCard,
        "LEFT DRIVE: --",
        10,
        34,
        180
    );
    rightDriveTempLabel = createLabel(
        temperatureCard,
        "RIGHT DRIVE: --",
        10,
        58,
        180
    );
    intakeTempLabel = createLabel(
        temperatureCard,
        "INTAKE: --",
        10,
        82,
        180
    );
    liftTempLabel = createLabel(
        temperatureCard,
        "LIFT: --",
        10,
        106,
        180
    );

    lv_obj_t* autonButton = lv_button_create(dashboardScreen);
    lv_obj_set_pos(autonButton, 5, 186);
    lv_obj_set_size(autonButton, 310, 48);
    styleButton(autonButton, COLOR_ACCENT);

    selectedAutonLabel = lv_label_create(autonButton);
    lv_label_set_text(selectedAutonLabel, "AUTON: NONE");
    lv_label_set_long_mode(selectedAutonLabel, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(selectedAutonLabel, 290);
    lv_obj_set_style_text_align(selectedAutonLabel, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(
        selectedAutonLabel,
        lv_color_hex(COLOR_BACKGROUND),
        LV_PART_MAIN
    );
    lv_obj_align(selectedAutonLabel, LV_ALIGN_CENTER, 0, 0);

    lv_obj_add_event_cb(
        autonButton,
        showAutonSelectorEvent,
        LV_EVENT_CLICKED,
        nullptr
    );

    lv_obj_t* distanceButton = lv_button_create(dashboardScreen);
    lv_obj_set_pos(distanceButton, 320, 186);
    lv_obj_set_size(distanceButton, 155, 48);
    styleButton(distanceButton, COLOR_CARD);
    lv_obj_t* distanceLabel = lv_label_create(distanceButton);
    lv_label_set_text(distanceLabel, "DISTANCE");
    lv_obj_set_style_text_color(distanceLabel, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_align(distanceLabel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(distanceButton, showDistanceEvent, LV_EVENT_CLICKED, nullptr);
}

void createDistanceScreen() {
    distanceScreen = lv_obj_create(nullptr);
    styleScreen(distanceScreen);
    lv_obj_t* header = createCard(distanceScreen, 5, 4, 470, 38);
    lv_obj_t* back = lv_button_create(header);
    lv_obj_set_pos(back, 5, 4);
    lv_obj_set_size(back, 80, 30);
    styleButton(back, COLOR_BORDER);
    lv_obj_t* backLabel = lv_label_create(back);
    lv_label_set_text(backLabel, "< BACK");
    lv_obj_set_style_text_color(backLabel, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_align(backLabel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(back, showDashboardEvent, LV_EVENT_CLICKED, nullptr);
    createLabel(header, "DISTANCE SENSORS", 110, 10, 340, COLOR_ACCENT);

    const std::array<const char*, 3> names{
        "FRONT / PORT 5", "LEFT / PORT 13", "RIGHT / PORT 10"
    };
    for (std::size_t i = 0; i < names.size(); ++i) {
        lv_obj_t* card = createCard(distanceScreen, 5, 49 + 53 * i, 470, 47);
        createLabel(card, names[i], 10, 14, 175, COLOR_MUTED);
        distanceLabels[i] = createLabel(card, "--", 190, 14, 270);
    }
    createLabel(distanceScreen, "Measured from sensor face. Display only.",
                10, 215, 460, COLOR_MUTED);
}

void createAutonScreen() {
    autonScreen = lv_obj_create(nullptr);
    styleScreen(autonScreen);

    lv_obj_t* header = createCard(autonScreen, 5, 4, 470, 38);

    lv_obj_t* backButton = lv_button_create(header);
    lv_obj_set_pos(backButton, 5, 4);
    lv_obj_set_size(backButton, 80, 30);
    styleButton(backButton, COLOR_BORDER);

    lv_obj_t* backLabel = lv_label_create(backButton);
    lv_label_set_text(backLabel, "< BACK");
    lv_obj_set_style_text_color(backLabel, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_align(backLabel, LV_ALIGN_CENTER, 0, 0);

    lv_obj_add_event_cb(
        backButton,
        showDashboardEvent,
        LV_EVENT_CLICKED,
        nullptr
    );

    lv_obj_t* title = createLabel(
        header,
        "AUTONOMOUS SELECTOR",
        100,
        11,
        270,
        COLOR_ACCENT
    );
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

    selectorMessageLabel = createLabel(
        autonScreen,
        "Tap a routine to select it",
        10,
        47,
        460,
        COLOR_MUTED
    );
    lv_obj_set_style_text_align(
        selectorMessageLabel,
        LV_TEXT_ALIGN_CENTER,
        LV_PART_MAIN
    );

    const std::size_t buttonCount = std::min(
        autonOptions.size(),
        MAX_AUTON_BUTTONS
    );

    autonButtons.clear();
    autonButtonLabels.clear();
    autonButtons.reserve(buttonCount);
    autonButtonLabels.reserve(buttonCount);

    for (std::size_t i = 0; i < buttonCount; ++i) {
        const int column = static_cast<int>(i % 2);
        const int row = static_cast<int>(i / 2);
        const int x = 10 + column * 235;
        const int y = 66 + row * 43;

        autonButtonIndices[i] = i;

        lv_obj_t* button = lv_button_create(autonScreen);
        lv_obj_set_pos(button, x, y);
        lv_obj_set_size(button, 225, 38);
        styleButton(button, COLOR_CARD);
        lv_obj_set_style_border_width(button, 2, LV_PART_MAIN);

        lv_obj_t* label = lv_label_create(button);
        lv_label_set_text(label, autonOptions[i].name);
        lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
        lv_obj_set_width(label, 205);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

        lv_obj_add_event_cb(
            button,
            selectAutonEvent,
            LV_EVENT_CLICKED,
            &autonButtonIndices[i]
        );

        autonButtons.push_back(button);
        autonButtonLabels.push_back(label);
    }

    runAutonButton = lv_button_create(autonScreen);
    lv_obj_set_pos(runAutonButton, 10, 199);
    lv_obj_set_size(runAutonButton, 460, 36);
    styleButton(runAutonButton, COLOR_ACCENT);

    runAutonButtonLabel = lv_label_create(runAutonButton);
    lv_label_set_text(runAutonButtonLabel, "HOLD TO RUN AUTON");
    lv_label_set_long_mode(runAutonButtonLabel, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(runAutonButtonLabel, 440);
    lv_obj_set_style_text_align(
        runAutonButtonLabel,
        LV_TEXT_ALIGN_CENTER,
        LV_PART_MAIN
    );
    lv_obj_align(runAutonButtonLabel, LV_ALIGN_CENTER, 0, 0);

    lv_obj_add_event_cb(
        runAutonButton,
        runAutonTestEvent,
        LV_EVENT_LONG_PRESSED,
        nullptr
    );

    updateAutonButtonStyles();
}

}  // namespace

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

void initialize(std::initializer_list<AutonOption> autonomousRoutines) {
    autonOptions.assign(autonomousRoutines.begin(), autonomousRoutines.end());
    selectedAutonIndex.store(0);
    autonTestRunning.store(false);
    autonTestRoutine = nullptr;

    createDashboardScreen();
    createAutonScreen();
    createDistanceScreen();

    updateSelectedAutonDisplay();
    updateAutonButtonStyles();

    updateDistanceSensors();
    lv_screen_load(dashboardScreen);
    lv_timer_create(dashboardUpdateTimer, DISPLAY_REFRESH_MS, nullptr);
}

void runSelectedAutonomous() {
    const AutonFunction routine = getSelectedAutonFunction();

    if (routine != nullptr) {
        routine();
    }
}

const char* getSelectedAutonomousName() {
    if (autonOptions.empty()) {
        return "None";
    }

    return autonOptions[getSafeSelectedAutonIndex()].name;
}

bool isAutonTestRunning() {
    return autonTestRunning.load();
}

}  // namespace robot_ui
