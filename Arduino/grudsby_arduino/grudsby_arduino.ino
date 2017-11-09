#include <ros.h>
#include <std_msgs/Float64.h>
#include <grudsby_lowlevel/ArduinoVel.h>
#include <grudsby_lowlevel/ArduinoResponse.h>
#include <SBUS.h>
#include <elapsedMillis.h>
#include <Servo.h> 

#include "settings.h"
#include "grudsby_motor.h"
// #include "Encoder.h"
#include "rc_control.h"

#include <Streaming.h>

//must include last
#include "grudsby_arduino.h"

using namespace grudsby;

void setup()
{
  //set up ros 
  //nh.initNode();
  //nh.subscribe(vel_sub);
  //nh.advertise(status_pub);

  // attachInterrupt(digitalPinToInterrupt(encoder1.channel_a_pin), ISR_1, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(encoder2.channel_a_pin), ISR_2, CHANGE);
  
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  leftMotor = new RCMotor(5);
  rightMotor = new RCMotor(6);

  autonomous = true;
  Serial.begin(115200);
  // init_timer(10);
}

void loop()
{
  moveGrudsby();
  //publishStatus();
  //nh.spinOnce();
}

// void ISR_1()
// {
//  encoder1.encoder_update(); 
// }
// void ISR_2()
// {
//  encoder2.encoder_update(); 
// }
// ISR(TIMER1_COMPA_vect) 
// {
//   encoder1.velocity_update(); 
//   encoder2.velocity_update();  
// } 

void velCallback(const grudsby_lowlevel::ArduinoVel& msg) {

  leftMotor->writeVal(msg.leftvel);
  rightMotor->writeVal(msg.rightvel);
}

void publishStatus() {
  // response_msg.leftvel = encoder1.get_velocity();
  // response_msg.rightvel = encoder2.get_velocity();
  // response_msg.leftpos = encoder1.get_position();
  // response_msg.rightpos = encoder2.get_position();
  response_msg.autonomous = autonomous;
  response_msg.kill = kill;

  status_pub.publish(&response_msg);
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
      nh.spinOnce();
      delay(20);
    }
    else if(!(kill) && !(autonomous)) {
      int rc_left_vel = rc.get_RC_left_motor_velocity();
      int rc_right_vel = rc.get_RC_right_motor_velocity();
      Serial<<"Left: "<<rc_left_vel<<"\tRight: "<<rc_right_vel<<endl;
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
