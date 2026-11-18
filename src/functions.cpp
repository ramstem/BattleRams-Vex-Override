#include "main.h"
#include "api.h"
#include "globals.h"
//#include "robot/monte.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/api.hpp"
#include "pros/apix.h"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rotation.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>
#include <string>
#include <iostream>

using namespace lemlib;

void lift() {
    liftMotors.move(127); // move front intake at full speed
}

void roller() {
    rollerMotors.move(127); // move front intake at full speed
}
