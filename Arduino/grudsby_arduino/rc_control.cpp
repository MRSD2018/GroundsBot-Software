#include <Arduino.h>
#include "SBUS.h"
#include "rc_control.h"

rc_control::rc_control() {
  x8r = new SBUS(Serial1);
  // begin the SBUS communication
  x8r->begin();
  //Serial.println("started receiver");
}

bool rc_control::read_signal(){
  //Serial.println(failSafe);
  if (x8r->read(&channels[0], &failSafe, &lostFrames))
  {
    return !(failSafe);
  }
}

//Making TRUE the default return for safety reasons. If the check fails for any reason everything dies.
bool rc_control::is_killed()
{
  if(channels[KILL_SWITCH] == 1811)
  {
    return false;
  }

  return true;
}

//Making FALSE the default return for safety reasons. If something goes wrong switch back to manual mode
bool rc_control::is_autonomous()
{
  if(channels[CONTROL_MODE] == 1811)
  {
    return true;
  }

  return false;
}

int rc_control::get_RC_left_motor_velocity()
{
  int compound_velocity = map(channels[THROTTLE], 172, 1811, 0, 255);
  
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

int rc_control::get_RC_right_motor_velocity()
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

int rc_control::get_raw_throttle()
{
  return (int) channels[THROTTLE];
}

int rc_control::get_raw_turn()
{
  return (int) channels[TURN];
}

int rc_control::get_raw_reverse()
{
  return (int) channels[REVERSE];
}

int rc_control::get_raw_kill()
{
  return (int) channels[KILL_SWITCH];
}

int rc_control::get_raw_mode()
{
  return (int) channels[CONTROL_MODE];
}
