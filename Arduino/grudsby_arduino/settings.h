#ifndef SETTINGS_H
#define SETTINGS_H

namespace grudsby {

const int MOTORA1 = 3;
const int MOTORB1 = 4;
const int ENABLE1 = 5;

const int DIR1 = 4;
const int PWM1 = 3;

const int MOTORA2 = 6;
const int MOTORB2 = 7;
const int ENABLE2 = 8;

const int DIR2 = 5;
const int PWM2 = 6;

const unsigned long PUBLISH_RATE = 50;  // millis
const unsigned long RC_TIMEOUT_LOST_SIGNAL = 2000;  // millis
const unsigned long AUTONOMOUS_DEBOUNCE_TIME = 200; // millis
const unsigned long KILL_DEBOUNCE_TIME = 200; // millis
const float WHEEL_DIST = 1;

const unsigned long AUTONOMOUS_CMD_TIMEOUT = 1000; // millis
}


#endif
