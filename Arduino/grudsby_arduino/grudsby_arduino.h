#ifndef GRUDSBY_ARDUINO_H
#define GRUDSBY_ARDUINO_H

#include <Arduino.h>
#include <ros.h>
#include "grudsby_motor.h"
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>

using namespace grudsby;

void initTimer();


void left_callback(const std_msgs::Float32& msg);
void right_callback(const std_msgs::Float32& msg);


void publishStatus();
bool moveGrudsby();

int32_t prevLPos = -999;
int32_t prevRPos = -999;

int32_t  prevLTimerPos =  0;
int32_t  prevRTimerPos =  0;

int32_t leftVel = 0;
int32_t rightVel = 0;

int leftAutoVal = 0;
int rightAutoVal = 0;
unsigned long lastPublish = 0;
unsigned long lastRCsignal = 0;
unsigned long lastAutonomous = 0;
unsigned long lastKill = 0;

bool killed;
bool autonomous;


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

grudsby_lowlevel::ArduinoResponse response_msg;

ros::Publisher response_pub("grudsby/arduino_response", &response_msg);
ros::Subscriber<std_msgs::Float32> left_sub("arduino/lwheel_vtarget", &left_callback);
ros::Subscriber<std_msgs::Float32> right_sub("arduino/rwheel_vtarget", &right_callback);

unsigned long lastCmdLeft = 0;
unsigned long lastCmdRight = 0;
int lastLeftVel = 0;
int lastRightVel = 0;

rc_control rc;

#endif
