#pragma once

namespace lift_calibration {
// Recover the sensor positions from the readings taken with the old scale.
inline constexpr double PREVIOUS_INCHES_PER_REVOLUTION = -1.770857753;
inline constexpr double FIRST_POINT_REVOLUTIONS = 6.0 / PREVIOUS_INCHES_PER_REVOLUTION;
inline constexpr double SECOND_POINT_REVOLUTIONS = 12.0 / PREVIOUS_INCHES_PER_REVOLUTION;
inline constexpr double FIRST_POINT_INCHES = 6.5;
inline constexpr double SECOND_POINT_INCHES = 14.5;

inline constexpr double heightInches(double revolutions) {
    // The sensor counts down as the lift rises. Extend the nearest segment
    // beyond measured travel so the readout can still show zero drift.
    if (revolutions >= FIRST_POINT_REVOLUTIONS) {
        return revolutions / FIRST_POINT_REVOLUTIONS * FIRST_POINT_INCHES;
    }
    return FIRST_POINT_INCHES +
           (revolutions - FIRST_POINT_REVOLUTIONS) /
               (SECOND_POINT_REVOLUTIONS - FIRST_POINT_REVOLUTIONS) *
               (SECOND_POINT_INCHES - FIRST_POINT_INCHES);
}
}
