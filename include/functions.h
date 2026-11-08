#pragma once

void chain_lift();
void chain_lift_reverse();
void chain_lift_brake();
void lift();
void roller();

void resetLocalization(float x, float y, float theta);
bool isHeadingNear(float headingDeg, float targetDeg, float tolDeg);
void localizeNow();