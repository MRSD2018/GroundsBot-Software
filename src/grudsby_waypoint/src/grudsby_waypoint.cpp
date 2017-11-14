#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include <sstream>

ros::Publisher waypoint_pub;

void findWaypointCallback(const nav_msgs::Odometry msg) 
{
  //Probably put these in a header
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::TransformStamped transformStamped;

  //Probably not right...
  transformStamped = tfBuffer.lookupTransform("tf/odom", "odometry", ros::Time(0));
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "waypoint_pub");
  ros::NodeHandle n;
  
  waypoint_pub = n.advertise<geometry_msgs::PoseStamped>("/goal_waypoint", 1000);

  ros::Subscriber local_odom_sub;
  //TODO: Fix topic name
  local_odom_sub = n.subscribe<nav_msgs::Odometry>("/key_vel", 100, findWaypointCallback)

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
