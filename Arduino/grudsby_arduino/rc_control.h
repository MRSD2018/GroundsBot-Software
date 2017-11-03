#ifndef RC_CONTROL_H
#define RC_CONTROL_H


#include <Arduino.h>
#include "SBUS.h"

// a SBUS object, which is on Teensy hardware
// serial port 1
static SBUS x8r(Serial1);

// channel, fail safe, and lost frames data
static uint16_t channels[16];
static uint8_t failSafe;
static uint16_t lostFrames = 0;

#define THROTTLE channels[2]
#define TURN channels[0]
#define KILL_SWITCH channels[5]
#define CONTROL_MODE channels[4]
#define REVERSE channels[6]

static int MIN_TURN = 10;


//void init();
bool is_killed();
bool is_autonomous();
int get_RC_left_motor_velocity();
int get_RC_right_motor_velocity();
int get_raw_throttle();
int get_raw_turn();
int get_raw_reverse();
int get_raw_kill();
int get_raw_mode();

#endif
