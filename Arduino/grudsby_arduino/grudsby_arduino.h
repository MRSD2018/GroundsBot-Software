#ifndef GRUDSBY_ARDUINO_H
#define GRUDSBY_ARDUINO_H

#include <Arduino.h>
#include <ros.h>
#include <grudsby_lowlevel/ArduinoVel.h>
#include <grudsby_lowlevel/ArduinoResponse.h>
#include "grudsby_motor.h"

using namespace grudsby;

void initTimer();

void velCallback(const grudsby_lowlevel::ArduinoVel& msg);


void publishStatus();

void moveGrudsby();

int32_t prevLPos = -999;
int32_t prevRPos = -999;

int32_t  prevLTimerPos =  0;
int32_t  prevRTimerPos =  0;

int32_t leftVel = 0;
int32_t rightVel = 0;

bool autonomous;
bool kill;

const float WHEELBASE_LEN = 0.508;
const float WHEEL_RAD = 0.127;


Encoder rightEncoder(3, 27);
Encoder leftEncoder(2, 24);

Motor* leftMotor;
Motor* rightMotor;

unsigned long last_lastEncMicros1 = 0;
unsigned long last_lastEncMicros0 = 0;
int32_t last_lPos = 0;
int32_t last_rPos = 0;

int publishVel = 1;

ros::NodeHandle nh;
ros::Subscriber<grudsby_lowlevel::ArduinoVel> vel_sub("/arduino/vel", &velCallback);

nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("grudsby/odometry", &odom_msg);

std_msgs::Int32 lwheel_msg;
ros::Publisher lwheel_pub("lwheel", &lwheel_msg);

std_msgs::Int32 rwheel_msg;
ros::Publisher rwheel_pub("rwheel", &rwheel_msg);

rc_control rc;

#endif