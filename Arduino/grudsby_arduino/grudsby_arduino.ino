#include <ros.h>
#include <std_msgs/Float64.h>
#include <grudsby_lowlevel/ArduinoVel.h>
#include <grudsby_lowlevel/ArduinoResponse.h>
#include <SBUS.h>
#include <elapsedMillis.h>
#include <Servo.h> 

#include "settings.h"
#include "grudsby_motor.h"
#include "rc_control.h"
// #include "Encoder.h"

//must include last
#include "grudsby_arduino.h"

using namespace grudsby;

<<<<<<< HEAD
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
=======
void setup()
{
>>>>>>> motor_classes
  //set up ros 
  nh.initNode();
  nh.subscribe(vel_sub);
  nh.advertise(status_pub);

  // attachInterrupt(digitalPinToInterrupt(encoder1.channel_a_pin), ISR_1, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(encoder2.channel_a_pin), ISR_2, CHANGE);
  
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  leftMotor = new RCMotor(5);
  rightMotor = new RCMotor(6);

  autonomous = true;

  rc_init();
  // init_timer(10);
}

<<<<<<< HEAD
void loop() {
  moveGrudsby();
=======
void loop()
{

  //Serial<<"Kill status: "<<is_killed(&channels[0])<<"\tAutonomous status: "<<is_autonomous(&channels[0])<<endl;
  //Serial<<"Raw kill: "<<channels[5]<<"\tRaw mode: "<<channels[4]<<endl;
  //Serial<<"Raw throttle: "<<channels[3]<<endl;
  //Serial<<get_RC_left_motor_velocity(&channels[0])<<endl;;
  
  
    //Serial<<"No kill, not auto"<<endl;
  if(x8r.read(&channels[0], &failSafe, &lostFrames))
  {
    if(is_killed(&channels[0]))
    {
      leftMotor->writeVal(0);
      rightMotor->writeVal(0);
    }
    else if(is_autonomous(&channels[0]))
    {
      nh.spinOnce();
      delay(20);
    }
    else if(!(is_killed(&channels[0])) && !(is_autonomous(&channels[0])))
    {
      //nh.spinOnce();
      int rc_left_vel = get_RC_left_motor_velocity(&channels[0]);
      int rc_right_vel = get_RC_right_motor_velocity(&channels[0]);
      //Serial.println(get_raw_throttle());
      //Serial<<"left: "<<rc_left_vel<<"\tright: "<<rc_right_vel<<endl;
      leftMotor->writeVal(rc_left_vel);
      rightMotor->writeVal(rc_right_vel);
      //delay(20);
      //Serial.println(channels[2]);
      //Serial.print("\t");
      //Serial.println(THROTTLE);
    }
    else
    {
      //have this twice because safety and spinning blade of death
      leftMotor->writeVal(0);
      rightMotor->writeVal(0);
    }
  }
  
  publishStatus();
  nh.spinOnce();

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
>>>>>>> motor_classes
}
