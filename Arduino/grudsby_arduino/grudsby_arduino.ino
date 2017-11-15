#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <grudsby_lowlevel/ArduinoVel.h>
#include <grudsby_lowlevel/ArduinoResponse.h>
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

void setup()
{
  //set up ros 
  nh.initNode();
  nh.subscribe(vel_sub);
  nh.advertise(lwheel_pub);
  nh.advertise(rwheel_pub);
  

  leftMotor = new RCMotor(8);
  rightMotor = new RCMotor(9);


  autonomous = true;
  Serial.begin(1000000);
}

void loop()
{
  moveGrudsby();
  publishStatus();
  nh.spinOnce();
  //delay(1);
}

ISR(TIMER2_COMPA_vect) {
  int32_t leftEncoderVal = leftEncoder.read();
  int32_t rightEncoderVal = rightEncoder.read();

  leftVel = leftEncoderVal - prevLTimerPos;
  rightVel = rightEncoderVal - prevRTimerPos;

  prevLTimerPos = leftEncoderVal;
  prevRTimerPos = rightEncoderVal;


// } 

void velCallback(const grudsby_lowlevel::ArduinoVel& msg) {

  leftMotor->writeVal(msg.leftvel);
  rightMotor->writeVal(msg.rightvel);
}

void publishStatus() {
  if (leftEncoder.read() !=  prevLPos) {
    lwheel_msg.data = leftEncoder.read();
    lwheel_pub.publish(&lwheel_msg);
    delay(10);
  }

  if (rightEncoder.read() != prevRPos) {
    rwheel_msg.data = rightEncoder.read();
    rwheel_pub.publish(&rwheel_msg);
  }

  prevLPos = leftEncoder.read();
  prevRPos = rightEncoder.read();

}


void moveGrudsby() {
  if(rc.read_signal()) {
    //Serial.println("read signal");
    kill = rc.is_killed();
    autonomous = rc.is_autonomous();
    //Serial<<"Kill: "<<rc.get_raw_kill()<<"\tAutonomous: "<<rc.get_raw_mode()<<endl;
    //Serial<<"Throttle: "<<rc.get_raw_throttle()<<"\tTurn: "<<rc.get_raw_turn()<<endl;
    if(kill) {
      leftMotor->writeVal(0);
      rightMotor->writeVal(0);
    }
    else if(autonomous) {
      //nh.spinOnce();
      delay(20);
    }
    else if(!(kill) && !(autonomous)) {
      int rc_left_vel = rc.get_RC_left_motor_velocity();
      int rc_right_vel = rc.get_RC_right_motor_velocity();
      //Serial<<"Left: "<<rc_left_vel<<"\tRight: "<<rc_right_vel<<endl;
      Serial.print(rc_left_vel);
      Serial.print(" ");
      Serial.print(rc_right_vel);
      Serial.println();
      leftMotor->writeVal(rc_left_vel);
      rightMotor->writeVal(rc_right_vel);
    }
    else {
      //have this twice because safety and spinning blade of death
      leftMotor->writeVal(0);
      rightMotor->writeVal(0);
    }
  }
  else {
    leftMotor->writeVal(0);
    rightMotor->writeVal(0);
  }
}

void initTimer(int freq) {
  TCCR2B = 0x00; // Disable Timer 2 until set up

  TCNT2 = 0; // reset timer count to 0
  TIFR2 = 0x00; // clear flags

  int register_value = (16000000/256/freq);
  OCR2A = register_value;

  TIMSK2 = 0x02; //match A interrupt enable
  TCCR2A = 0x02; //wave gen CTC to OCRA
  TCCR2B = 0x06; // prescaler to 256

}