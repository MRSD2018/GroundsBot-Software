#include <ros.h>
#include <std_msgs/Float64.h>
#include <grudsby_lowlevel/ArduinoVel.h>
#include <SBUS.h>
#include <elapsedMillis.h>

#include "settings.h"
#include "grudsby_motor.h"
#include "rc_control.h"

using namespace grudsby;




void velCallback(const grudsby_lowlevel::ArduinoVel& msg) {
  writeDirPWM(msg.leftvel, msg.rightvel);
}

void publishResponse() {

}

ros::NodeHandle nh;
ros::Subscriber<grudsby_lowlevel::ArduinoVel> vel_sub("/arduino/vel", &velCallback);
void setup()
{
  //set up ros 
  nh.initNode();
  nh.subscribe(vel_sub);

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

}

void loop()
{
  nh.spinOnce();
  delay(20);
}
