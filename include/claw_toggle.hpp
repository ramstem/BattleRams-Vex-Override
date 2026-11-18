#pragma once

// One direction at a time. Held buttons do not repeatedly toggle the claw.
class ClawToggle {
public:
    int update(bool forwardHeld, bool reverseHeld) {
        const bool forwardPressed = forwardHeld && !previousForward;
        const bool reversePressed = reverseHeld && !previousReverse;
        previousForward = forwardHeld;
        previousReverse = reverseHeld;
        if (forwardHeld && reverseHeld) direction = 0;
        else if (forwardPressed) direction = direction == 1 ? 0 : 1;
        else if (reversePressed) direction = direction == -1 ? 0 : -1;
        return direction * 127;
    }

    // Preserve button history so a held button cannot restart after a pause.
    void stop() { direction = 0; }

private:
    int direction = 0;
    bool previousForward = false;
    bool previousReverse = false;
};
