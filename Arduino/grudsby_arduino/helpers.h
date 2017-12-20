#ifndef GRUDSBY_HELPERS_H
#define GRUDSBY_HELPERS_H

#include "settings.h"
#include <Arduino.h>
#include "grudsby_motor.h"
namespace grudsby {

void writeCmdVel(float vel, float angle, Motor& left_motor, Motor& right_motor);

}

//rostopic pub /topic_name std_msgs/String hello

#endif