#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "navsat_conversions.h"
#include <sensor_msgs/NavSatFix.h>
#include <sstream>

ros::Publisher waypoint_pub;

void findWayPointCallback(const sensor_msgs::NavSatFix& msg)
{
  double grudsby_lat = msg->latitude;
  double grudsby_long = msg->longitude;
  double grudsby_alt = msg->altitude;

  geometry_msgs::PoseStamped goal;
  
  goal.header.stamp = ros:time::now();
  goal.header.frame_id = "utm";


  //PSEUDO CODE
  //TODO: MAKE THIS REAL!!
  if ( grudsby_lat == goal_lat && grudsby_long = goal_long )
  {
    goal_lat = next_waypoint_lat;
    goal_long = next_waypoint_long;
    goal_alt = next_waypoint_alt;

    goal_easting_x = 0;
    goal_northing_y = 0;
    std::string utm_zone_tmp;

    RobotLocalization::NavsatConversions::LLtoUTM(goal_lat, goal_long, goal_northing_y, goal_easting_x, utm_zone_tmp);

    goal.pose.position.x = goal_easting_x;
    goal.pose.position.y = goal_northing_y;
    goal.pose.position.z = goal_alt;
  }

  waypoint_pub.plublish(goal)
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "waypoint_pub");
  ros::NodeHandle n;
  
  waypoint_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);

  ros::Subscriber navsat_sub;
  navsat_sub = n.subscribe<sensor_msgs::NavSatFix>("gps/fix", 100, findWaypointCallback)

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
