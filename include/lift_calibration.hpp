#pragma once

namespace lift_calibration {
// With the previous scale, 6 inches on the display was 7 inches of actual travel.
// Keep upward travel positive; the sensor counts down as the lift rises.
inline constexpr double INCHES_PER_SENSOR_REVOLUTION = -1.770857753;
}
