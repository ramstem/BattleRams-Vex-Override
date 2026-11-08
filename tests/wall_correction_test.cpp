// Run from the project root:
// g++ -std=c++23 -Itests/wall_mock -Iinclude src/wall_correction.cpp \
//     tests/wall_correction_test.cpp -o /tmp/wall_correction_test
// /tmp/wall_correction_test
#include "globals.h"
#include "functions.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

void reset() {
    wall_mock::tick = 0;
    wall_mock::connected = wall_mock::disabled = false;
    wall_mock::autonomous = true;
    wall_mock::heading = 0;
    wall_mock::drift = wall_mock::turn = wall_mock::disableLater = wall_mock::busyLater = false;
    chassis = {};
    distanceFront.readings = {584}; // 23 inches, rounded to millimeters.
    distanceLeft.readings = distanceRight.readings = {9999};
}
int main() {
    struct Example { int heading; WallSensor sensor; WallAxis axis; int wall; int inches; };
    // Physical robot at (10,20), sensor face 3 right / 5 forward.
    const Example examples[] = {
        {0, WallSensor::Front, WallAxis::Y, 48, 23},
        {0, WallSensor::Left, WallAxis::X, -48, 61},
        {0, WallSensor::Right, WallAxis::X, 48, 35},
        {90, WallSensor::Front, WallAxis::X, 48, 33},
        {90, WallSensor::Left, WallAxis::Y, 48, 31},
        {90, WallSensor::Right, WallAxis::Y, -48, 65},
        {180, WallSensor::Front, WallAxis::Y, -48, 63},
        {180, WallSensor::Left, WallAxis::X, 48, 41},
        {180, WallSensor::Right, WallAxis::X, -48, 55},
        {270, WallSensor::Front, WallAxis::X, -48, 53},
        {270, WallSensor::Left, WallAxis::Y, -48, 71},
        {270, WallSensor::Right, WallAxis::Y, 48, 25}
    };
    for (auto e : examples) {
        reset();
        const bool x = e.axis == WallAxis::X;
        chassis.pose = {x ? 12.f : 10.f, x ? 20.f : 22.f, static_cast<float>(e.heading)};
        auto& sensor = e.sensor == WallSensor::Front ? distanceFront :
                       e.sensor == WallSensor::Left ? distanceLeft : distanceRight;
        sensor.readings = {static_cast<int>(std::round(e.inches * 25.4))};
        assert(correctPositionFromWall({e.sensor, e.axis, static_cast<double>(e.wall), 3, 5}));
        assert(chassis.changes == 1);
        assert(std::abs(chassis.pose.x - 10) < 0.03 && std::abs(chassis.pose.y - 20) < 0.03);
        assert(chassis.pose.theta == e.heading);
    }

    const WallCheckpoint front{WallSensor::Front, WallAxis::Y, 48, 3, 5};
    auto rejected = [&](WallCheckpoint c = {WallSensor::Front, WallAxis::Y, 48, 3, 5}) {
        assert(!correctPositionFromWall(c));
        assert(chassis.changes == 0);
    };
    reset(); chassis.busy = true; rejected();
    reset(); wall_mock::busyLater = true; rejected();
    reset(); wall_mock::connected = wall_mock::disabled = true; rejected();
    reset(); wall_mock::connected = true; wall_mock::autonomous = false; rejected();
    reset(); wall_mock::connected = wall_mock::disableLater = true; rejected();
    reset(); wall_mock::heading = std::numeric_limits<double>::infinity(); rejected();
    reset(); chassis.pose.x = std::numeric_limits<float>::quiet_NaN(); rejected();
    reset(); distanceFront.readings = {9999}; rejected();
    reset(); distanceFront.readings = {std::numeric_limits<int>::max()}; rejected();
    reset(); distanceFront.readings = {19}; rejected();
    reset(); distanceFront.readings = {2001}; rejected();
    reset(); distanceFront.readings = {584, 584, 600, 584}; rejected();
    reset(); wall_mock::drift = true; rejected();
    reset(); wall_mock::turn = true; rejected();
    reset(); chassis.pose.theta = 10; rejected();
    reset(); auto c = front; c.wallCoordinate = -48; rejected(c);
    reset(); c = front; c.maxCorrection = 1; rejected(c);
    reset(); c = front; c.maxCorrection = 0; rejected(c);
    reset(); c = front; c.sensorRight = std::numeric_limits<double>::infinity(); rejected(c);
    reset(); c = front; c.sensor = static_cast<WallSensor>(99); rejected(c);
    reset(); c = front; c.axis = static_cast<WallAxis>(99); rejected(c);
    // Heading near the wrap boundary still faces +Y, and projects correctly.
    reset(); chassis.pose.theta = 359; assert(correctPositionFromWall(front));
    assert(chassis.pose.theta == 359 && chassis.pose.x == 10);
    assert(std::abs(chassis.pose.y - 19.96) < 0.02);
    // Robot-specific mounts at heading 0; wall locations relative to start.
    reset(); chassis.pose = {0, 2, 0}; distanceFront.readings = {508};
    assert(correctPositionFromWall(WallSensor::Front, WallAxis::Y, 22.25));
    assert(chassis.pose.x == 0 && std::abs(chassis.pose.y) < 0.001);
    reset(); chassis.pose = {2, 0, 0}; distanceLeft.readings = {508};
    assert(correctPositionFromWall(WallSensor::Left, WallAxis::X, -25));
    assert(std::abs(chassis.pose.x) < 0.001 && chassis.pose.y == 0);
    reset(); chassis.pose = {2, 0, 0}; distanceRight.readings = {508};
    assert(correctPositionFromWall(WallSensor::Right, WallAxis::X, 25));
    assert(std::abs(chassis.pose.x) < 0.001 && chassis.pose.y == 0);
    reset(); assert(!correctPositionFromWall(static_cast<WallSensor>(99), WallAxis::Y, 25));
    assert(chassis.changes == 0);
    std::cout << "38 wall correction scenarios passed\n";
}
