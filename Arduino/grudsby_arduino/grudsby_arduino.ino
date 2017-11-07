#include <ros.h>
#include <std_msgs/Float64.h>
#include <grudsby_lowlevel/ArduinoVel.h>
#include <SBUS.h>
#include <elapsedMillis.h>
#include <Streaming.h>

#include "settings.h"
#include "grudsby_motor.h"
#include "rc_control.h"

using namespace grudsby;




void velCallback(const grudsby_lowlevel::ArduinoVel& msg) {

  writeDirPWM(msg.leftvel, msg.rightvel);
}

void publishResponse() {

}

void moveGrudsby() {
  if(x8r.read(&channels[0], &failSafe, &lostFrames)) {
    if(is_killed(&channels[0])) {
      writeDirPWM(0,0);
    }
    else if(is_autonomous(&channels[0])) {
      nh.spinOnce();
      delay(20);
    }
    else if(!(is_killed(&channels[0])) && !(is_autonomous(&channels[0]))) {
      int rc_left_vel = get_RC_left_motor_velocity(&channels[0]);
      int rc_right_vel = get_RC_right_motor_velocity(&channels[0]);
      Serial<<"left: "<<rc_left_vel<<"\tright: "<<rc_right_vel<<endl;
      writeDirPWM(rc_left_vel, rc_right_vel);
    }
    else {
      //have this twice because safety and spinning blade of death
      writeDirPWM(0,0);
    }
  }
}

ros::NodeHandle nh;
ros::Subscriber<grudsby_lowlevel::ArduinoVel> vel_sub("/arduino/vel", &velCallback);
void setup() {
  //set up ros 
  //nh.initNode();
  //nh.subscribe(vel_sub);

  //set up pins
  // pinMode(MOTORA1, OUTPUT);
  // pinMode(MOTORB1, OUTPUT);
  // pinMode(MOTORA2, OUTPUT);
  // pinMode(MOTORB2, OUTPUT);
  // pinMode(ENABLE1, OUTPUT);
  // pinMode(ENABLE2, OUTPUT);

  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  Serial.begin(115200);

  rc_init();
}

void loop() {
  moveGrudsby();
}
