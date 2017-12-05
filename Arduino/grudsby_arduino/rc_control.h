#ifndef RC_CONTROL_H
#define RC_CONTROL_H


#include <Arduino.h>
#include "SBUS.h"
// #include <StandardCplusplus.h>
// #include <vector>

class rc_control
{
private:
    SBUS *x8r;

    // channel, fail safe, and lost frames data
    uint16_t channels[16];
    uint8_t failSafe;
    uint16_t lostFrames = 0;

    const int THROTTLE = 1;
    const int TURN = 0;
    const int KILL_SWITCH = 5;
    const int CONTROL_MODE = 4;
    const int REVERSE = 6;
    const int MOWER = 7;
    const int MIN_VEL= 40;

public:

    rc_control();
    bool read_signal();
    bool is_killed();
    bool is_autonomous();
    bool is_mower_on();
    int get_RC_left_motor_velocity();
    int get_RC_right_motor_velocity();
    int get_raw_throttle();
    int get_raw_turn();
    int get_raw_reverse();
    int get_raw_kill();
    int get_raw_mode();
    void get_RC_motor_outputs(int &outL, int &outR);
    void get_RC_exponential_outputs(int &outL, int &outR);
    void get_RC_weenie_outputs(int &outL, int &outR);
    bool get_RC_mower_output();

};

#endif
