#pragma once

namespace lift_calibration {
// A measured 6.00-inch rise read 8.36 inches with the previous scale.
// Apply the measured correction and keep upward travel positive.
inline constexpr double INCHES_PER_SENSOR_REVOLUTION = (6.0 / -2.837) * (6.0 / 8.36);
}
