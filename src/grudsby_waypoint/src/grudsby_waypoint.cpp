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
#include <cstdlib> // for exit()

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
      std::cerr << "Uh oh, mower_path could not be opened for reading!" << std::endl;
      exit(1);
  }

  std::string line;
  while ( std::getline(infile, line) )
  {
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
    }
  } 
}

void findWaypointCallback(const sensor_msgs::NavSatFix& msg)
{
  double grudsby_lat = msg.latitude;
  double grudsby_long = msg.longitude;
  double grudsby_alt = msg.altitude;

  geometry_msgs::PoseStamped goal;
  
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "utm";


  //PSEUDO CODE
  //TODO: MAKE THIS REAL!!
  /*if ( grudsby_lat == goal_lat && grudsby_long = goal_long )
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
  }*/

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
  std::cout << "done parsing" << std::endl;
  std::cout.precision(17);
  for ( const waypoint &point : goals )
  {
    std::cout << point.latitude << ", " << point.longitude << std::endl;
  }

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
