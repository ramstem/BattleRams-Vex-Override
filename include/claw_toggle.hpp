#pragma once

// R2 toggles forward. L2 runs reverse from press until release.
class ClawToggle {
public:
    int update(bool forwardHeld, bool reverseHeld) {
        const bool forwardPressed = forwardHeld && !previousForward;
        const bool reversePressed = reverseHeld && !previousReverse;
        previousForward = forwardHeld;
        previousReverse = reverseHeld;
        if (forwardHeld && reverseHeld) {
            stop();
            return 0;
        }
        if (!reverseHeld) reversing = false;
        if (reversePressed) {
            forwardOn = false;
            reversing = true;
        }
        if (reversing) return -127;
        if (forwardPressed) forwardOn = !forwardOn;
        return forwardOn ? 127 : 0;
    }

    // Preserve button history so a held button cannot restart after a pause.
    void stop() {
        forwardOn = false;
        reversing = false;
    }

private:
    bool forwardOn = false;
    bool reversing = false;
    bool previousForward = false;
    bool previousReverse = false;
};
