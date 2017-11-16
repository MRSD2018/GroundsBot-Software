#include <ros.h>

#include <nav_msgs/Odometry.h>

#define ENCODER_OPTIMIZE_INTERRUPTS

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
  nh.advertise(odom_pub);
  


  leftMotor = new RCMotor(22);
  rightMotor = new RCMotor(23);


  autonomous = true;
  Serial.begin(1000000);

  initTimer2();



}

void loop()
{
  moveGrudsby();
  lPos = leftEncoder.read();
  rPos = rightEncoder.read();
  publishStatus();
  // nh.spinOnce();
  //delay(1);

//   Serial.print(leftVel);
//   Serial.print(" ");
//   Serial.print(-rightVel);
//   Serial.print(" ");
//   Serial.print(lPos);
//   Serial.print(" ");
//   Serial.println(-rPos);
}

ISR(TIMER2_COMPA_vect) {

  leftVel = lPos - prevLTimerPos;
  rightVel = rPos - prevRTimerPos;

  prevLTimerPos = lPos;
  prevRTimerPos = rPos;

} 

void velCallback(const grudsby_lowlevel::ArduinoVel& msg) {

  leftMotor->writeVal(msg.leftvel);
  rightMotor->writeVal(msg.rightvel);
}

void publishStatus() {
  // wheel rads = ticks_per_sec / ticks_per_rev  * 2pi
  
  double leftRadPerSec = ((leftVel * 250) / 4096.0) * 2 * 3.14157 ; // * 250 becuz sampled at 250 hz
  double rightRadPerSec = ((rightVel * 250) / 4096.0) * 2 * 3.14157;

  double x_vel = (WHEEL_RAD/2.0) * (leftRadPerSec + rightRadPerSec);
  double z_rot = (WHEEL_RAD / WHEELBASE_LEN) * (rightRadPerSec - leftRadPerSec);

  odom_msg.twist.twist.linear.x = x_vel;
  odom_msg.twist.twist.angular.z = z_rot;
  
  odom_pub.publish(&odom_msg);

  Serial.print(x_vel);
  Serial.print(" ");
  Serial.println(z_rot);

  // if (leftEncoder.read() !=  prevLPos) {
  //   lwheel_msg.data = leftEncoder.read();
  //   lwheel_pub.publish(&lwheel_msg);
  //   delay(10);
  // }

  // if (rightEncoder.read() != prevRPos) {
  //   rwheel_msg.data = rightEncoder.read();
  //   rwheel_pub.publish(&rwheel_msg);
  // }

  // prevLPos = leftEncoder.read();
  // prevRPos = rightEncoder.read();

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