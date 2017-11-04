#include <Arduino.h>
#include "SBUS.h"
#include "rc_control.h"

void rc_init() {
  // begin the SBUS communication
  x8r.begin();
  Serial.println("started receiver");
}

//Making TRUE the default return for safety reasons. If the check fails for any reason everything dies.
bool is_killed()
{
  if(channels[KILL_SWITCH] == 1811)
  {
    return false;
  }

  return true;
}

//Making FALSE the default return for safety reasons. If something goes wrong switch back to manual mode
bool is_autonomous()
{
  if(channels[CONTROL_MODE] == 1811)
  {
    return true;
  }

  return false;
}

int get_RC_left_motor_velocity(uint16_t *channels)
{
  //Serial.println("getting velocity");
  int compound_velocity = map(channels[THROTTLE], 172, 1811, 0, 255);
  //Serial.println(channels[THROTTLE]);
  //Serial.println(compound_velocity);
  //yes, this is correct. When joystick is to the right, subtract from left velocity
  //Don't let it go negative
  //MIN_TURN is the mininum velocity a wheel can take when turning. 
  int right_val = max(0, map(channels[TURN], 985, 1811, 0, compound_velocity - MIN_TURN));

  int left_velocity = max(0, compound_velocity - right_val);

  if(channels[REVERSE] == 1811)
  {
    return left_velocity * -1;
  }

  return left_velocity;
}

int get_RC_right_motor_velocity(uint16_t *channels)
{
  int compound_velocity = map(channels[THROTTLE], 172, 1811, 0, 255);
  
  //yes, this is correct. When joystick is to the left, subtract from right velocity
  //Don't let it go negative
  //MIN_TURN is the mininum velocity a wheel can take when turning. 
  int left_val = max(0, map(channels[TURN], 985, 172, 0, compound_velocity - MIN_TURN));

  int right_velocity = max(0, compound_velocity - left_val);

  if(channels[REVERSE] == 1811)
  {
    return right_velocity * -1;
  }

  return right_velocity;
}

int get_raw_throttle()
{
  int throttle = 0;
  throttle = THROTTLE;
  return throttle;
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
