#include <Arduino.h>
#include "SBUS.h"
#include "rc_control.h"

// using namespace std;

rc_control::rc_control() {
  x8r = new SBUS(Serial2);
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


void rc_control::get_RC_motor_outputs(int &outL, int &outR) 
{
  int joy_y_val = map(channels[THROTTLE], 1811, 172, -255, 255);
  int joy_x_val = map(channels[TURN], 172, 1811, -255, 255);

  float premix_left;
  float premix_right;

  float pivotlimit = 100; 

  if (joy_y_val >= 0) {
    //Forward
    premix_left = (joy_x_val>=0)? 255 : (255 + joy_x_val);
    premix_right = (joy_x_val>=0)? (255 - joy_x_val) : 255;
  }
  else {
    //Reverse
    premix_left = (joy_x_val>=0)? (255 - joy_x_val) : 255;
    premix_right = (joy_x_val>=0)? 255: 255+joy_x_val;
  }

  premix_left = premix_left * (joy_y_val/255.0);
  premix_right = premix_right * (joy_y_val/255.0);

  float pivSpeed = joy_x_val;
  float pivScale = (abs(joy_y_val)>pivotlimit)? 0.0 : (1.0 - abs(joy_y_val)/pivotlimit);

  int mixed_left = (1.0 - pivScale) * premix_left + pivScale * pivSpeed;
  int mixed_right = (1.0 - pivScale) * premix_right + pivScale * -pivSpeed;


  outL = mixed_left;
  outR = mixed_right;


}

void rc_control::get_RC_exponential_outputs(int &outL, int &outR)
{
  int left_unscaled;
  int right_unscaled;

  rc_control::get_RC_motor_outputs(left_unscaled, right_unscaled);

  long left_scaled = left_unscaled^5;
  long right_scaled = right_unscaled^5;

  outL = map(left_unscaled, (-255)^5, 255^5, -255, 255);
  outR = map(right_unscaled, (-255)^5, 255^5, -255, 255);


}

int rc_control::get_RC_left_motor_velocity()
{

  int left_velocity = 0;
  int left_val = 0;

  int compound_velocity = map(channels[THROTTLE], 1811, 172, -255, 255);
  //Serial.println(compound_velocity);
  if(compound_velocity > MIN_VEL*-1 && compound_velocity < MIN_VEL){
    compound_velocity = 0;
  }

  //yes, this is correct. When joystick is to the right, subtract from left velocity
  if(compound_velocity > 0) {
    left_val = max(0, map(channels[TURN], 985, 172, 0, compound_velocity - MIN_VEL));
    left_velocity = max(0, compound_velocity - left_val);
  }
  else if(compound_velocity < 0) {
    left_val = min(0, map(channels[TURN], 985, 172, 0, compound_velocity + MIN_VEL));
    left_velocity = min(0, compound_velocity - left_val);
  }
  else if (compound_velocity == 0) {
    //zero-point turn
    left_velocity = map(channels[TURN], 172, 1811, -127, 127);
    if(left_velocity > MIN_VEL*-1 && left_velocity < MIN_VEL){
      left_velocity = 0;
    }
  }

  return left_velocity;
}

int rc_control::get_RC_right_motor_velocity()
{
  int right_velocity = 0;
  int right_val = 0;

  int compound_velocity = map(channels[THROTTLE], 1811, 172, -255, 255);

  if(compound_velocity > MIN_VEL*-1 && compound_velocity < MIN_VEL){
    compound_velocity = 0;
  }
  
  //yes, this is correct. When joystick is to the left, subtract from right velocity
  if(compound_velocity > 0) {
    right_val = max(0, map(channels[TURN], 985, 1811, 0, compound_velocity - MIN_VEL));
    right_velocity = max(0, compound_velocity - right_val);
  }
  else if(compound_velocity < 0) {
    right_val = min(0, map(channels[TURN], 985, 1811, 0, compound_velocity + MIN_VEL));
    right_velocity = min(0, compound_velocity - right_val);
  }
  else if(compound_velocity == 0) {
    //zero-point turn
    right_velocity = map(channels[TURN], 172, 1811, 127, -127);
    if(right_velocity > MIN_VEL*-1 && right_velocity < MIN_VEL){
      right_velocity = 0;
    }
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
