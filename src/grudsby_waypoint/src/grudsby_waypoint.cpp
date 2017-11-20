#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "navsat_conversions.h"
#include <sensor_msgs/NavSatFix.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <math.h>
#include <ros/console.h>// for logging

ros::Publisher waypoint_pub;


struct waypoint
{
  double latitude;
  double longitude;
  double altitude;
};

static std::vector<waypoint> goals;

void parseKMLFile()
{
  std::ifstream infile("/home/adam/GroundsBot/GroundsBot-Software/src/grudsby_waypoint/src/mower_path.kml");

  if (!infile)
  {
      // Print an error and exit
      ROS_ERROR("Cannot open mower_path file for parsing! No goal waypoints created!");
  }

  std::string line;
  while ( std::getline(infile, line) )
  {
    ROS_INFO("Parsing mower_path file to get waypoints.");
    std::stringstream ss;
    ss << line;
    char ch;
    ss >> ch;
    if ( ch == ('<') ) 
      continue;
    
    ss.putback(ch);
    while ( ss )
    {
      char count = 0;
      std::string combined_coords;
      std::stringstream combined_ss;
      std::getline(ss, combined_coords, ' ');
      combined_ss << combined_coords;
      waypoint tmp;
      std::string s_coord;
      while ( std::getline(combined_ss, s_coord, ',') )
      {
        if ( count == 0 ) tmp.longitude = atof( s_coord.c_str() );
        else if ( count == 1 ) tmp.latitude = atof( s_coord.c_str() );
        else if ( count == 2 ) tmp.altitude = atof( s_coord.c_str() );
        count ++;
      }
      goals.push_back(tmp);
      ROS_INFO("Added GPS waypoint: %f, %f, %f", tmp.latitude, tmp.longitude, tmp.altitude);
    }
  } 
}

double deg2rad(double deg) {
  return deg * (M_PI/180);
}

bool inThreshold(double lat, double lon, double goal_lat, double goal_lon)
{
  int R = 6371; // Radius of the earth in km
  double dLat = deg2rad(goal_lat - lat);  // deg2rad below
  double dLon = deg2rad(goal_lon - lon); 
  double a = 
    sin(dLat/2) * sin(dLat/2) +
    cos(deg2rad(lat)) * cos(deg2rad(goal_lat)) * 
    sin(dLon/2) * sin(dLon/2)
    ; 
  double c = 2 * atan2(sqrt(a), sqrt(1-a)); 
  double d = R * c; // Distance in km
  
  if ( d <= 0.0003 ) {
    return true;
  }
  return false;
  
  //Dumb version incase smart version doesn't work
  /*double glat_plus = goal_lat + 0.000003;
  double glat_minus = goal_lat - 0.000003;
  double glon_plus = goal_lon + 0.000003;
  double glon_minus = goal_lon - 0.000003;

  if ( lat < glat_plus && lat > glat_minus &&
        lon < glon_plus && lon > glon_minus)
  {
    return true;
  }
  return false;*/
}

void findWaypointCallback(const sensor_msgs::NavSatFix& msg)
{
  ROS_INFO("Building new goal message");
  double grudsby_lat = msg.latitude;
  double grudsby_long = msg.longitude;
  double grudsby_alt = msg.altitude;

  geometry_msgs::PoseStamped goal;
  
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "utm";
  goal.pose.orientation.w = 1.0;

  double goal_lat = goals.front().latitude;
  double goal_long = goals.front().longitude;

  if ( inThreshold(grudsby_lat, grudsby_long, goal_lat, goal_long) )
  {
    ROS_INFO("Updating goal waypoint");
    goals.erase( goals.begin() );
    goal_lat = goals.front().latitude;
    goal_long = goals.front().longitude;
  }

  double goal_easting_x = 0;
  double goal_northing_y = 0;
  std::string utm_zone_tmp;

  RobotLocalization::NavsatConversions::LLtoUTM(goal_lat, goal_long, goal_northing_y, goal_easting_x, utm_zone_tmp);

  goal.pose.position.x = goal_easting_x;
  goal.pose.position.y = goal_northing_y;
  goal.pose.position.z = grudsby_alt;
  ROS_INFO("UTM Goal: Northing: %f, Easting: %f", goal_northing_y, goal_easting_x);

  waypoint_pub.publish(goal);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "waypoint_pub");
  ros::NodeHandle n;
  
  waypoint_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);

  ros::Subscriber navsat_sub;
  navsat_sub = n.subscribe("gps/fix", 100, findWaypointCallback);

  parseKMLFile();
  ROS_INFO("Waypoint file done parsing");

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
