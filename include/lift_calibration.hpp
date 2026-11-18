#pragma once

namespace lift_calibration {
// Signed carriage travel per rotation-sensor revolution. Leave 0 until measured;
// the dashboard then displays signed shaft revolutions instead of guessed inches.
// Example calibration: measure a 6-inch rise and divide 6 by displayed rev.
// A negative rev reading while raising requires a negative conversion factor.
inline constexpr double INCHES_PER_SENSOR_REVOLUTION = 0.0;
}
