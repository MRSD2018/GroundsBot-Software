#include <ros.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <grudsby_lowlevel/ArduinoResponse.h>
#include <grudsby_lowlevel/ArduinoVel.h>

#include <SBUS.h>
#include <elapsedMillis.h>
#include <Servo.h>

#include "settings.h"
#include "grudsby_motor.h"
#include <Encoder.h>
#include "rc_control.h"


//must include last
#include "grudsby_arduino.h"

using namespace grudsby;

//12000 ticks per second = 2.92968 revs/sec max speed  -- set in roboclaw

void left_callback(const std_msgs::Float32& msg) {
  lastCmdLeft = millis();
  float scaled = msg.data * 1000;
  float maxvel = (12000/4096.0) * WHEEL_RAD* 2 * 3.14159 * 1000;
  int val = map(scaled, -maxvel, maxvel, -255, 255);
  leftAutoVal = val;
}

void right_callback(const std_msgs::Float32& msg) {
  lastCmdRight = millis();
  float scaled = msg.data*1000;
  float maxvel = (12000/4096.0) * WHEEL_RAD* 2 * 3.14159 * 1000;
  int val = map(scaled, -maxvel, maxvel, -255, 255);
  rightAutoVal = val;
}

void setup() {
  //set up ros 
  nh.initNode();
  nh.subscribe(left_sub);
  nh.subscribe(right_sub);
  nh.advertise(response_pub);

  leftMotor = new RCMotor(4);
  rightMotor = new RCMotor(6);
  leftMotor->detachServo();
  rightMotor->detachServo();
  pinMode(MOWER_RELAY, OUTPUT);
  digitalWrite(MOWER_RELAY, LOW);
  killed = true;
  autonomous = false;
  mowerEnabled = false;
}

void loop()
{
  moveGrudsby();  
  publishStatus();

  nh.spinOnce();
}


bool moveGrudsby() {
  if(rc.read_signal()) {
    lastRCsignal = millis();
    //Serial.println("read signal");
    if (killed) { // Killed
      if(rc.is_killed()) {
        lastKill = millis();
      }
      if ((millis()-lastKill) > KILL_DEBOUNCE_TIME) {
        killed = false;
        leftMotor->attachServo();
        rightMotor->attachServo();
        lastKill = millis();
        return false;
      }
    }
    else { // Not killed
      if(!rc.is_killed()) {
        lastKill = millis();
      }

      if (mowerEnabled) {
        if (rc.is_mower_on()) {
          lastMower = millis();
        }
        if ((millis() - lastMower) > MOWER_DEBOUNCE_TIME) {
          mowerEnabled = false;
          digitalWrite(MOWER_RELAY, LOW);
          lastMower = millis();
          return false;
        }
        // Placeholder for mower enabled
      }
      else {
        if (!rc.is_mower_on()) {
          lastMower = millis();
        }
        if ((millis() - lastMower) > MOWER_DEBOUNCE_TIME) {
          mowerEnabled = true;
          digitalWrite(MOWER_RELAY, HIGH);
          lastMower = millis();
          return false;
        }
        // Placeholder for mower disabled
      }
            
      if ((millis()-lastKill) > KILL_DEBOUNCE_TIME) {
        killed = true;
        leftMotor->detachServo();
        rightMotor->detachServo();
        autonomous = false;
        leftMotor->writeVal(0);
        rightMotor->writeVal(0);
        mowerEnabled = false;
        digitalWrite(MOWER_RELAY, LOW);
        lastKill = millis();
        lastRightVel = 0;
        lastLeftVel = 0;
        return false;
      }
      if (autonomous) { // Autonomous mode
        if (rc.is_autonomous()) {
           lastAutonomous = millis();
        }
        if ((millis()-lastAutonomous) > AUTONOMOUS_DEBOUNCE_TIME) {
          autonomous = false;
          lastAutonomous = millis();
          return false;
        }
        //  Placeholder for moving autonomously

        if ((millis() - lastCmdLeft) > AUTONOMOUS_CMD_TIMEOUT) {
          if (lastLeftVel > 0) {
            lastLeftVel -= 1; 
            delay(15);
          }
          if (lastLeftVel < 0) {
            lastLeftVel += 1;
            delay(15);
          }
          leftMotor->writeVal(lastLeftVel);
        }
        else 
        {
          leftMotor->writeVal(leftAutoVal);
          lastLeftVel = leftAutoVal;
        }

        if ((millis() - lastCmdRight) > AUTONOMOUS_CMD_TIMEOUT) {
          if (lastRightVel > 0) {
            lastRightVel -= 1;
            delay(15);
          }
          if (lastRightVel < 0) {
            lastRightVel += 1;
            delay(15);
          }
          rightMotor->writeVal(lastRightVel);
        }
        else 
        {
          rightMotor->writeVal(rightAutoVal);
          lastRightVel = rightAutoVal;
        }
      }
      else { // Manual mode
        if (!rc.is_autonomous()) {
          lastAutonomous = millis();
        }
        if ((millis()-lastAutonomous) > AUTONOMOUS_DEBOUNCE_TIME){
          autonomous = true;
          lastAutonomous = millis();
          return false;
        }
        // Placeholder for manual
        int velL; 
        int velR;
        rc.get_RC_weenie_outputs(velL, velR);
        leftMotor->writeVal(velL);
        rightMotor->writeVal(velR);
        lastRightVel = 0;
        lastLeftVel = 0;
      }
    }
  }
  else {
    if ((millis() - lastRCsignal) > RC_TIMEOUT_LOST_SIGNAL) {
      killed = true;
      leftMotor->detachServo();
      rightMotor->detachServo(); 
      autonomous = false;
      leftMotor->writeVal(0);
      rightMotor->writeVal(0);
      mowerEnabled = false;
      digitalWrite(MOWER_RELAY, LOW);
      return false;
    }
  }
  return true;
}


void publishStatus() {
  // wheel rads = ticks_per_sec / ticks_per_rev  * 2pi
  
  //extern from encoder.h
  //if (publishVel%20 == 0) {
  if ((millis()-lastPublish) > PUBLISH_RATE) {
    lastPublish = millis();
    noInterrupts();
    unsigned long lastEncMicros1_curr = lastEncMicros1;
    unsigned long lastEncMicros0_curr = lastEncMicros0;
    int32_t lPos = leftEncoder.read();
    int32_t rPos = rightEncoder.read();
    interrupts();

    // //pub vels //
    float leftRadPerSec = 0;
    float rightRadPerSec = 0;

    int lPosDiff = int(lPos) - int(last_lPos);
    int rPosDiff = int(rPos) - int(last_rPos);

    if (lastEncMicros1_curr != last_lastEncMicros1) {
      leftRadPerSec = ((lPosDiff) / (float(lastEncMicros1_curr - last_lastEncMicros1))) * 2 * 3.14159 ; 
    }
    if (lastEncMicros0_curr != last_lastEncMicros0){
      rightRadPerSec = ((rPosDiff) / (float(lastEncMicros0_curr - last_lastEncMicros0))) * 2 * 3.14159;
    }

    float x_vel = (WHEEL_RAD/2.0) * (leftRadPerSec + rightRadPerSec) * (1e6/ 4096.0);
    float z_rot = (WHEEL_RAD / WHEELBASE_LEN) * (rightRadPerSec - leftRadPerSec) * (1e6/4096.0);

    last_lastEncMicros0 = lastEncMicros0_curr;
    last_lastEncMicros1 = lastEncMicros1_curr;
    last_lPos = lPos;
    last_rPos = rPos;

    response_msg.linearX = x_vel;
    response_msg.angularZ = z_rot;
    response_msg.encoderLeft = lPos;
    response_msg.encoderRight = rPos;
    response_msg.velLeft = leftRadPerSec;
    response_msg.velRight = rightRadPerSec;

    response_pub.publish(&response_msg);
    //reset timeout count
    publishVel = 1;
  }
  publishVel++;
}

// Not used
void initTimer2() {
  noInterrupts();
  TCCR2B = 0x00; // Disable Timer 2 until set up

  TCNT2 = 0; // reset timer count to 0
  TIFR2 = 0x00; // clear flags

  OCR2A = 255; //max 8 bit int
  
  TCCR2A = 0; //reset tccr2a

  TIMSK2 |= _BV(OCIE2A); //match A interrupt enable
  TCCR2A |= _BV(WGM21) | _BV(WGM20) | _BV(COM2A1) | _BV(COM2A0); //wavegen to fastpwm and set oc2a on compare match
  //TCCR2B |= (1 << CS21);
  //TCCR2B = 4;
  TCCR2B |= _BV(CS22) | _BV(CS21) | _BV(WGM22); //prescaler to 256 and wavegen to pwm mode

  interrupts();
}
