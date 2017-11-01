#include <ros.h>
#include <std_msgs/Float64.h>
#include <grudsby_lowlevel/ArduinoVel.h>

#include "settings.h"
#include "grudsby_motor.h"


ros::NodeHandle nh;
//ros::Subscriber<grudsby_lowlevel::ArduinoVel> s("/arduino/vel", &velCallback);


using namespace grudsby;

void setup()
{
  nh.initNode();
  //nh.subscribe(s);

  pinMode(MOTORA1, OUTPUT);
  pinMode(MOTORB1, OUTPUT);
  pinMode(MOTORA2, OUTPUT);
  pinMode(MOTORB2, OUTPUT);
  pinMode(ENABLE1, OUTPUT);
  pinMode(ENABLE2, OUTPUT);
}

void loop()
{
  nh.spinOnce();
  //delay(10);
}
