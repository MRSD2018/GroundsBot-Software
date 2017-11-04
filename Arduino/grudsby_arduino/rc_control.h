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

static int THROTTLE = 2;
static int TURN = 0;
static int KILL_SWITCH = 5;
static int CONTROL_MODE = 4;
static int REVERSE = 6;

static int MIN_TURN = 10;


void rc_init();
bool is_killed();
bool is_autonomous();
int get_RC_left_motor_velocity(uint16_t *channels);
int get_RC_right_motor_velocity(uint16_t *channels);
int get_raw_throttle();
int get_raw_turn();
int get_raw_reverse();
int get_raw_kill();
int get_raw_mode();

#endif
