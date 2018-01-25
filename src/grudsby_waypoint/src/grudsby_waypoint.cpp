#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
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
#include <ros/console.h>  // for logging

ros::Publisher waypoint_pub;

ros::Publisher stop_pub;
ros::Time last_stop_update;
bool wait_at_waypoint;
std::string mower_path;

struct Waypoint
{
  double latitude;
  double longitude;
  double altitude;
};

static std::vector<Waypoint> goals;

void parseKMLFile()
{
  std::ifstream infile(mower_path.c_str());


  if (!infile)
  {
    // Print an error and exit
    ROS_ERROR("Cannot open mower_path file for parsing at file location %s! No goal waypoints created!", mower_path.c_str());
  }

  std::string line;
  while (std::getline(infile, line))
  {
    ROS_INFO("Parsing mower_path file to get waypoints.");
    std::stringstream ss;
    ss << line;
    char ch;
    ss >> ch;
    if (ch == ('<'))
    {
      continue;
    }

    ss.putback(ch);
    while (ss)
    {
      char count = 0;
      std::string combined_coords;
      std::stringstream combined_ss;
      std::getline(ss, combined_coords, ' ');
      combined_ss << combined_coords;
      Waypoint tmp;
      std::string s_coord;
      while (std::getline(combined_ss, s_coord, ','))
      {
        if (count == 0)
        {
          tmp.longitude = atof(s_coord.c_str());
        }
        else if (count == 1)
        {
          tmp.latitude = atof(s_coord.c_str());
        }
        else if (count == 2)
        {
          tmp.altitude = atof(s_coord.c_str());
        }
        count++;
      }
      goals.push_back(tmp);
      ROS_INFO("Added GPS waypoint: %f, %f, %f", tmp.latitude, tmp.longitude, tmp.altitude);
    }
  }
}

double deg2rad(double deg)
{
  return deg * (M_PI / 180);
}

bool inThreshold(double lat, double lon, double goal_lat, double goal_lon)
{
  int r = 6371.0;                            // Radius of the earth in km
  double d_lat = deg2rad(goal_lat - lat);  // deg2rad below
  double d_lon = deg2rad(goal_lon - lon);
  double a =
      sin(d_lat / 2) * sin(d_lat / 2) + cos(deg2rad(lat)) * cos(deg2rad(goal_lat)) * sin(d_lon / 2) * sin(d_lon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double d = r * c;  // Distance in km
  ROS_ERROR("Dist: %f", d);
  return d <= 0.0009;

  // Dumb version incase smart version doesn't work
  /*double glat_plus = goal_lat + 0.000003;
  double glat_minus = goal_lat - 0.000003;
  double glon_plus = goal_lon + 0.000003;
  double glon_minus = goal_lon - 0.00set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
0003;

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

  if (!goals.empty())
  {
    double goal_lat = goals.front().latitude;
    double goal_long = goals.front().longitude;

    if (inThreshold(grudsby_lat, grudsby_long, goal_lat, goal_long))
    {
      ROS_INFO("Updating goal waypoint");
      if (goals.size() > 1)
      {
        if (goals.front().altitude > 0.5)
        {
          last_stop_update = ros::Time::now();
        }
        goals.erase(goals.begin());
        goal_lat = goals.front().latitude;
        goal_long = goals.front().longitude;
      }
      else
      {
        last_stop_update = ros::Time::now();
        ROS_WARN("No new waypoints. Repeating previous waypoint.");
      }
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

    ros::Duration wait = ros::Time::now() - last_stop_update;
    std_msgs::Bool stop;
    stop.data = (wait_at_waypoint && (wait.toSec() < 2.5));  //NOLINT
    stop_pub.publish(stop);
  }
  else
  {
    ROS_WARN("No waypoints in vector.");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_pub");
  ros::NodeHandle n;

  ROS_INFO("Waypoint file done parsing");

  if (!n.getParam("grudsby_waypoint/wait_at_point", wait_at_waypoint)) 
  {
    wait_at_waypoint = true;
  }
  if (!n.getParam("grudsby_waypoint/mower_path_file", mower_path))
  {
    mower_path = "/home/nvidia/GroundsBot-Software/data/mower_path.kml";
  }
  else
  {
    ROS_INFO("Loaded mower path from param");
  }
  
  parseKMLFile();

  waypoint_pub = n.advertise<geometry_msgs::PoseStamped>("/goal", 1000);

  stop_pub = n.advertise<std_msgs::Bool>("/grudsby/stop", 100);

  ros::Subscriber navsat_sub;
  navsat_sub = n.subscribe("/fix", 100, findWaypointCallback);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
