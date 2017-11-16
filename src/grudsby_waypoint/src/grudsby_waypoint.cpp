#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "navsat_conversions.h"
#include <geodesy/wgs84.h>
#include <geodesy/utm.h>
#include <sstream>

ros::Publisher waypoint_pub;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "waypoint_pub");
  ros::NodeHandle n;
  
  waypoint_pub = n.advertise<geometry_msgs::PoseStamped>("/goal_waypoint", 1000);

  //ros::Subscriber local_odom_sub;
  //TODO: Fix topic name
  //local_odom_sub = n.subscribe<nav_msgs::Odometry>("/robot/localization", 100, findWaypointCallback)

  double latitude = 40.44455571433333;
  double longitude = -79.94060748566666;
  double easting = 0;
  double northing= 0;
  std::string utm_zone_tmp;

  RobotLocalization::NavsatConversions::LLtoUTM(latitude, longitude, northing, easting, utm_zone_tmp);

  //geodesy::UTMPoint::UTMPoint() *converted_coord;

  //geodesy::fromMsg(*wgs_coords, *converted_coord);

  std::cout<<"Easting: "<<easting<<"\t Northing: "<<northing<<std::endl;


  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
