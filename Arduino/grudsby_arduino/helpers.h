#ifndef GRUDSBY_HELPERS_H
#define GRUDSBY_HELPERS_H

#include "settings.h"
#include <Arduino.h>

namespace grudsby {

void writeCmdVel(float vel, float angle, Motor& left_motor, Motor& right_motor);
 	// dx = (l + r) / 2
    // dr = (r - l) / w
	float speed_wish_right = (angle * grudsby::WHEEL_DIST)/2.0 + vel;
	float speed_wish_left = cmd_vel.speed*2 - speed_wish_right;
}

#endif