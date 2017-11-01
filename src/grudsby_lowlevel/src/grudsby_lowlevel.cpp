#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <grudsby_lowlevel/ArduinoVel.h>

ros::Publisher arduino_vel_pub;

///TODO CHANGE INPUT MESSAGE TYPE
void arduinoVelCallback(const grudsby_lowlevel::ArduinoVelConstPtr& msg) 
{
  grudsby_lowlevel::ArduinoVel send_message;

  ///TODO CHANGE ME
  send_message.leftvel = 0; 
  send_message.rightvel = 0;
  send_message.header.stamp = ros::Time::now();
  
}

void arduinoTeleopCallback(const geometry_msgs::TwistConstPtr& msg) 
{
  grudsby_lowlevel::ArduinoVel send_message;

  //max -255 to 255
  //forward
  if (msg->linear.x > 0) {
    send_message.leftvel = 200;
    send_message.rightvel = 200;
  }
  else if (msg->linear.x < 0) {
    send_message.leftvel = -200;
    send_message.rightvel = -200;
  }
  else {
    //turn left
    if (msg->angular.z > 0) 
    {
      send_message.leftvel = -100;
      send_message.rightvel = 100;
    }
    else if (msg->angular.z < 0){
      send_message.leftvel = 100;
      send_message.rightvel = -100;
    }
    else {
      send_message.leftvel = 0;
      send_message.rightvel = 0;
    }
  }
  
  send_message.header.stamp = ros::Time::now();
  arduino_vel_pub.publish(send_message);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "arduino_comms");
  ros::NodeHandle n;
  
  arduino_vel_pub = n.advertise<grudsby_lowlevel::ArduinoVel>("/arduino/vel", 1000);

  ros::Subscriber arduino_teleop_sub;
  bool keyboard_teleop;
  // if (ros::param::get("keyboard_teleop", keyboard_teleop ))
  // {
  //   arduino_teleop_sub = n.subscribe<geometry_msgs::Twist>("/key_vel", 100, arduinoTeleopCallback);
  // }
  arduino_teleop_sub = n.subscribe<geometry_msgs::Twist>("/key_vel", 100, arduinoTeleopCallback);

  ros::Rate loop_rate(10);

  ros::Subscriber sub = n.subscribe("/grudsby/vels", 1000, arduinoVelCallback);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
