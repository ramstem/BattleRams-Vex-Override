// g++ -std=c++23 -Iinclude tests/claw_toggle_test.cpp -o /tmp/claw_toggle_test
#include "claw_toggle.hpp"
#include <cassert>
#include <iostream>

int main() {
    ClawToggle claw;
    assert(claw.update(false, false) == 0);
    assert(claw.update(true, false) == 127);
    for (int i = 0; i < 100; ++i) assert(claw.update(true, false) == 127);
    assert(claw.update(false, false) == 127);
    assert(claw.update(true, false) == 0);
    claw.update(false, false);
    assert(claw.update(false, true) == -127);
    assert(claw.update(false, true) == -127);
    assert(claw.update(false, false) == 0);
    assert(claw.update(false, true) == -127);
    assert(claw.update(false, false) == 0);
    assert(claw.update(true, false) == 127);
    claw.update(false, false);
    assert(claw.update(false, true) == -127); // Opposite press reverses.
    assert(claw.update(false, false) == 0); // Release stops instead of resuming forward.
    assert(claw.update(true, false) == 127);
    assert(claw.update(true, true) == 0); // Both held stops.
    assert(claw.update(false, true) == 0); // Releasing one does not restart.
    claw.update(false, false);
    assert(claw.update(false, true) == -127);
    claw.stop();
    assert(claw.update(false, true) == 0); // Held across a pause stays stopped.
    claw.update(false, false);
    assert(claw.update(false, true) == -127);
    ClawToggle restarted;
    assert(restarted.update(false, false) == 0);
    assert(restarted.update(true, true) == 0);
    std::cout << "Claw press, hold, reverse, stop, pause and restart checks passed\n";
}
