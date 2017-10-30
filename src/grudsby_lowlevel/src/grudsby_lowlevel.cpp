#include "ros/ros.h"
#include "std_msgs/String.h"
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

int main(int argc, char **argv)
{

  ros::init(argc, argv, "arduino-comms");
  ros::NodeHandle n;

  ros::Publisher arduino_vel_pub = n.advertise<grudsby_lowlevel::ArduinoVel>("arduino/vel", 1000);

  ros::Rate loop_rate(10);

  ros::Subscriber sub = n.subscribe("/grudsby/vels", 1000, arduinoVelCallback);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
