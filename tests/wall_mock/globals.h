#pragma once
#include <algorithm>
#include <cstdint>
#include <vector>

namespace lemlib {
struct Pose { float x, y, theta; };
}
namespace wall_mock {
inline int tick = 0;
inline bool connected = false, disabled = false, autonomous = true;
inline double heading = 0;
inline bool drift = false, turn = false, disableLater = false, busyLater = false;
}
namespace pros {
inline void delay(int) { ++wall_mock::tick; }
namespace competition {
inline bool is_connected() { return wall_mock::connected; }
inline bool is_disabled() {
    return wall_mock::disabled || (wall_mock::disableLater && wall_mock::tick >= 2);
}
inline bool is_autonomous() { return wall_mock::autonomous; }
}
struct Distance {
    std::vector<int> readings{1000};
    int get() { return readings[std::min<std::size_t>(wall_mock::tick, readings.size() - 1)]; }
};
}
inline pros::Distance distanceFront, distanceLeft, distanceRight;
struct MockImu { double get_heading() { return wall_mock::heading; } };
inline MockImu imu;
struct MockChassis {
    lemlib::Pose pose{10, 22, 0};
    bool busy = false;
    int changes = 0;
    bool isInMotion() { return busy || (wall_mock::busyLater && wall_mock::tick >= 2); }
    lemlib::Pose getPose() {
        auto result = pose;
        if (wall_mock::tick >= 2) {
            if (wall_mock::drift) result.x += 1;
            if (wall_mock::turn) result.theta += 2;
        }
        return result;
    }
    void setPose(float x, float y, float theta) { pose = {x, y, theta}; ++changes; }
};
inline MockChassis chassis;
