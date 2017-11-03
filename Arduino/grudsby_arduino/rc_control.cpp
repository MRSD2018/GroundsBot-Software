#include <Arduino.h>
#include "SBUS.h"
#include "rc_control.h"

void rc_init() {
  // begin the SBUS communication
  x8r.begin();
}

//Making TRUE the default return for safety reasons. If the check fails for any reason everything dies.
bool is_killed()
{
  if(KILL_SWITCH == 1811)
  {
    return false;
  }

  return true;
}

//Making FALSE the default return for safety reasons. If something goes wrong switch back to manual mode
bool is_autonomous()
{
  if(CONTROL_MODE == 1811)
  {
    return true;
  }

  return false;
}

int get_RC_left_motor_velocity()
{
  int compound_velocity = map(THROTTLE, 172, 1811, 0, 255);
  //yes, this is correct. When joystick is to the right, subtract from left velocity
  int right_val = map(TURN, 985, 1811, 0, 255);

  int left_velocity = max(0, compound_velocity - right_val);

  if(REVERSE == 1811)
  {
    return left_velocity * -1;
  }

  return left_velocity;
}

int get_RC_right_motor_velocity()
{
  int compound_velocity = map(channels[2], 172, 1811, -255, 255);
  //yes, this is correct. When joystick is to the left, subtract from right velocity
  int left_val = map(TURN, 985, 1811, 0, 255);

  int right_velocity = max(0, compound_velocity - left_val);

  if(REVERSE == 1811)
  {
    return right_velocity * -1;
  }

  return right_velocity;
}

int get_raw_throttle()
{
  return THROTTLE;
}

int get_raw_turn()
{
  return TURN;
}

int get_raw_reverse()
{
  return REVERSE;
}

int get_raw_kill()
{
  return KILL_SWITCH;
}

int get_raw_mode()
{
  return CONTROL_MODE;
}
