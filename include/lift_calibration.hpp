#pragma once

namespace lift_calibration {
// Calibration from the fully lowered position: 6.00 inches of carriage rise
// produced -2.837 sensor revolutions. The negative scale makes raising positive.
inline constexpr double INCHES_PER_SENSOR_REVOLUTION = 6.0 / -2.837;
}
