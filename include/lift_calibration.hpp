#pragma once

namespace lift_calibration {

// Scoring heights in inches above the fully lowered position.
inline constexpr double SCORING_HEIGHTS_INCHES[] = {
    10.0, 15.5, 21.0, 26.5, 32.0, 37.5
};

// Physical travel ceiling from the fully lowered position.
inline constexpr double MAX_SAFE_HEIGHT_INCHES = 49.0;

// Recover the sensor positions from the readings taken with the old scale.
inline constexpr double PREVIOUS_INCHES_PER_REVOLUTION = -1.770857753;
inline constexpr double FIRST_POINT_REVOLUTIONS = 6.0 / PREVIOUS_INCHES_PER_REVOLUTION;
inline constexpr double SECOND_POINT_REVOLUTIONS = 12.0 / PREVIOUS_INCHES_PER_REVOLUTION;
inline constexpr double FIRST_POINT_INCHES = 6.5;
inline constexpr double SECOND_POINT_INCHES = 14.5;
inline constexpr double MAX_CALIBRATED_HEIGHT_INCHES = SECOND_POINT_INCHES;

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
