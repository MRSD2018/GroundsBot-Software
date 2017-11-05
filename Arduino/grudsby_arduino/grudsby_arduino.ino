#include <ros.h>
#include <std_msgs/Float64.h>
#include <grudsby_lowlevel/ArduinoVel.h>
#include <grudsby_lowlevel/ArduinoResponse.h>
#include <SBUS.h>
#include <elapsedMillis.h>

#include "settings.h"
#include "grudsby_motor.h"
#include "rc_control.h"
#include "Encoder.h"

using namespace grudsby;

////////STATUS VARS////////////
bool autonomous;
bool kill;
////////////CALLBACKS////////////////////////////////

void velCallback(const grudsby_lowlevel::ArduinoVel& msg) {

  writeDirPWM(msg.leftvel, msg.rightvel);
}



///////////ENCODERS/////////////////////////

Encoder encoder1(2, 4);
Encoder encoder2(3, 5);

void ISR_1()
{
 encoder1.encoder_update(); 
}
void ISR_2()
{
 encoder2.encoder_update(); 
}
ISR(TIMER1_COMPA_vect) 
{
  encoder1.velocity_update(); 
  encoder2.velocity_update();  
} 


////////////ROSSERIAL/////////////////////

ros::NodeHandle nh;
ros::Subscriber<grudsby_lowlevel::ArduinoVel> vel_sub("/arduino/vel", &velCallback);
grudsby_lowlevel::ArduinoResponse response_msg;
ros::Publisher status_pub("/arduino/status", &response_msg);

void publishStatus() {
  response_msg.leftvel = encoder1.get_velocity();
  response_msg.rightvel = encoder2.get_velocity();
  response_msg.leftpos = encoder1.get_position();
  response_msg.rightpos = encoder2.get_position();
  response_msg.autonomous = autonomous;
  response_msg.kill = kill;

  status_pub.publish(response_msg);
}

void setup()
{
  //set up ros 
  nh.initNode();
  nh.subscribe(vel_sub);
  nh.advertise(status_pub);


  //set up pins
  // pinMode(MOTORA1, OUTPUT);
  // pinMode(MOTORB1, OUTPUT);
  // pinMode(MOTORA2, OUTPUT);
  // pinMode(MOTORB2, OUTPUT);
  // pinMode(ENABLE1, OUTPUT);
  // pinMode(ENABLE2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoder1.channel_a_pin), ISR_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2.channel_a_pin), ISR_2, CHANGE);
  
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  autonomous = true;

  rc_init();
  init_timer(10);
}

void loop()
{
  if(x8r.read(&channels[0], &failSafe, &lostFrames)){
    kill = is_killed();
    is_autonomous();
  }
  
  publishStatus();
  nh.spinOnce();

}
