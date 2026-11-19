#pragma once

#include <cstddef>

namespace lift_calibration {

// Scoring heights in inches above the fully lowered position.
inline constexpr double SCORING_HEIGHTS_INCHES[] = {
    10.0, 15.5, 21.0, 26.5, 32.0, 37.5
};

// Physical travel ceiling from the fully lowered position.
inline constexpr double MAX_SAFE_HEIGHT_INCHES = 49.0;

namespace detail {
// Convert readings from the display used during the tape measurements back
// to sensor turns. The former display changed slope at 6.5 inches.
constexpr double revolutionsFromPriorDisplay(double inches) {
    constexpr double priorScale = -1.770857753;
    constexpr double lowerPoint = 6.0 / priorScale;
    constexpr double upperPoint = 12.0 / priorScale;
    if (inches <= 6.5) return inches / 6.5 * lowerPoint;
    return lowerPoint + (inches - 6.5) / (14.5 - 6.5) * (upperPoint - lowerPoint);
}

struct CalibrationPoint {
    double revolutions;
    double tapeInches;
};

// Tape measurements after the mechanical adjustments, in order of lift travel.
inline constexpr CalibrationPoint POINTS[] = {
    {0.0, 0.0},
    {revolutionsFromPriorDisplay(6.0), 8.75},
    {revolutionsFromPriorDisplay(12.0), 15.25},
    {revolutionsFromPriorDisplay(24.0), 27.5},
    {revolutionsFromPriorDisplay(36.0), 40.5}
};
inline constexpr auto LAST_POINT = sizeof(POINTS) / sizeof(POINTS[0]) - 1;
} // namespace detail

inline constexpr double MAX_CALIBRATED_HEIGHT_INCHES =
    detail::POINTS[detail::LAST_POINT].tapeInches;

inline constexpr double heightInches(double revolutions) {
    // Turns decrease as the lift rises. Outside the measured range, the
    // display extends the nearest segment. Preset moves use the measured range.
    std::size_t upper = 1;
    while (upper < detail::LAST_POINT && revolutions < detail::POINTS[upper].revolutions) {
        ++upper;
    }
    const auto& start = detail::POINTS[upper - 1];
    const auto& end = detail::POINTS[upper];
    const double fraction = (revolutions - start.revolutions) /
                            (end.revolutions - start.revolutions);
    return start.tapeInches + fraction * (end.tapeInches - start.tapeInches);
}
} // namespace lift_calibration
