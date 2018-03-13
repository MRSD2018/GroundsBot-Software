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
#include "grudsby_sweeping/SimpleLatLng.h"
#include "grudsby_sweeping/MowingPlan.h"
#include "tf/transform_listener.h"
#include <tf/transform_datatypes.h>
#include <png.h>
#include "Vector2.hpp"
#include <algorithm>
#include <cmath>


ros::Publisher waypoint_pub;
ros::Publisher map_lines_reset_pub;

ros::Publisher stop_pub;
ros::Time last_stop_update;
ros::Time last_goal_update;
bool wait_at_waypoint;
std::string mower_path;
double goal_publish_rate = 2; // Rate in hertz to publish new goals.

double resolution_;

double negate_;

std::string map_directory_;

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
  //ROS_ERROR("Dist: %f", d);
  return d <= 0.0003;

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


void outputBorder(double newLat, double newLng, double oldLat, double oldLng, double resolution,  double negate,  std::string map_directory)
{
  // Parse region
  std::vector<std::vector<double>> parsedRegion;
  std::vector<double> oldLatLng;
  oldLatLng.push_back(oldLng);
  oldLatLng.push_back(oldLat);
  parsedRegion.push_back(oldLatLng);
  std::vector<double> newLatLng;
  newLatLng.push_back(newLng);
  newLatLng.push_back(newLat);
  parsedRegion.push_back(newLatLng);
  
  static tf::TransformListener listener;

  if (!listener.waitForTransform("/map", "/utm", ros::Time::now(), ros::Duration(1.0)))
  { 
    return;
  }
  double max_x = -9999999999;
  double max_y = -9999999999;
  double min_x = 9999999999;
  double min_y = 9999999999; 
  for (std::vector<double> &latlng : parsedRegion)
  {
    double goal_easting_x = 0;
    double goal_northing_y = 0;
    std::string utm_zone_tmp;
    //ROS_ERROR("input lat lng:%f,%f",goal_lat,goal_long);
    RobotLocalization::NavsatConversions::LLtoUTM(
      latlng[1],
      latlng[0],
      goal_northing_y,
      goal_easting_x,
      utm_zone_tmp
    );
    
    geometry_msgs::PoseStamped coord, coord_in_map;
    coord.pose.position.x = goal_easting_x;
    coord.pose.position.y = goal_northing_y;
    coord.pose.position.z = 0;
    coord.pose.orientation.w = 1; 
    coord.header.frame_id = "/utm";
    try
    {
      listener.transformPose("/map", coord, coord_in_map);
      latlng[0] = coord_in_map.pose.position.x;
      latlng[1] = coord_in_map.pose.position.y;
      //ROS_ERROR("Lat %f Lng %f",latlng[0],latlng[1]);
      max_x = std::max(max_x,latlng[0]);
      min_x = std::min(min_x,latlng[0]);
      max_y = std::max(max_y,latlng[1]);
      min_y = std::min(min_y,latlng[1]);
    
      //ROS_ERROR("MIN %f %f   MAX %f %f",min_x,min_y,max_x,max_y);
    }
    catch (tf::TransformException ex)  //NOLINT
    {
      ROS_ERROR("%s", ex.what());
    }
  }
  ROS_INFO("Got Transforms"); 
  // Push all edges out by "outer_edge_buffer"
  parsedRegion.push_back(parsedRegion[0]); 
  
  max_x += 4;
  min_x -= 4;
  max_y += 4;
  min_y -= 4;      
  double width = (max_x-min_x)/resolution;
  double height = (max_y-min_y)/resolution;
  ROS_INFO("Outputting yaml");
  // Output YAML
  std::ofstream file_out((map_directory + "/map_lines.yaml").c_str(), std::ios::binary);
  file_out << "image: map_lines.png" << std::endl;
  file_out << "resolution: " << resolution << std::endl;
  file_out << "origin: [" << min_x << ", " << min_y << ", 0.0]" << std::endl;
  file_out << "occupied_thresh: 0.0" << std::endl;
  file_out << "free_thresh: 1.0" << std::endl;
  file_out << "mode: raw" << std::endl;
  file_out << "negate: " << negate << std::endl;
  file_out.close();
  
  ROS_INFO("Building map");
  //ROS_ERROR("MIN %f %f   MAX %f %f",min_x,min_y,max_x,max_y);
  int no_cols = std::ceil((max_x-min_x)/resolution);
  int no_rows = std::ceil((max_y-min_y)/resolution);
  const int init_val = 255;
  std::vector< std::vector<uint8_t> > mat;
  mat.resize(no_rows, std::vector<uint8_t>(no_cols, init_val) ); 
  for (int x = 0; x < no_cols; x++)//min_x; x < max_x; x += resolution)
  {
    for (int y = 0; y < no_rows; y++)//= min_y; y < max_y; y += resolution)
    {
      // Output PNG
      std::vector<double> test_point;
      test_point.push_back(min_x + x*resolution);
      test_point.push_back(max_y - y*resolution);
      double minDist = 99999999999;
      for (int i = 0; i < parsedRegion.size()-1; i++) 
      {
        double dist = sqrt(pow((parsedRegion[i][0]-test_point[0]),2.0) + pow((parsedRegion[i][1]-test_point[1]),2.0)); 
        minDist = std::min(minDist, dist);
        Vector2 fullVec;
        fullVec.X = parsedRegion[i+1][0] - parsedRegion[i][0];
        fullVec.Y = parsedRegion[i+1][1] - parsedRegion[i][1];
        Vector2 unitFull = Vector2::Normalized(fullVec);  
        Vector2 testVec;
        testVec.X = test_point[0] - parsedRegion[i][0];
        testVec.Y = test_point[1] - parsedRegion[i][1];
        double dotted = Vector2::Dot(testVec,unitFull);
        if ((dotted > 0) && (dotted < Vector2::Magnitude(fullVec)))
        {
          Vector2 orthVec = testVec - dotted*unitFull;
          minDist = std::min(minDist, Vector2::Magnitude(orthVec));
        } 
      }      
      double mapped = std::min(pow(minDist,0.5)*5.0,5.0)*10.0;
      mat[y][x] = static_cast<uint8_t>(mapped);
      
    }
  }
  
  ROS_INFO("Output the map to png");
 
  int width_mat = mat[0].size();
  int height_mat = mat.size();
 
  FILE *fp = fopen((map_directory+"/map_lines.png").c_str(), "wb");
  if(!fp) ROS_ERROR("Failed to open png file.");

  png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if (!png) ROS_ERROR("File to write is not a png file.");

  png_infop info = png_create_info_struct(png);
  if (!info) ROS_ERROR("Failure greating png info struct");

  if (setjmp(png_jmpbuf(png))) ROS_ERROR("Couldn't set jmpbuf.");

  png_init_io(png, fp);

  // Output is 8bit depth, RGBA format.
  png_set_IHDR(
    png,
    info,
    width_mat, height_mat,
    8,
    PNG_COLOR_TYPE_RGBA,
    PNG_INTERLACE_NONE,
    PNG_COMPRESSION_TYPE_DEFAULT,
    PNG_FILTER_TYPE_DEFAULT
  );
  png_write_info(png, info);

  png_byte color_type = PNG_COLOR_TYPE_PALETTE;
  png_byte bit_depth = 16;
  png_bytep *row_pointers;
  row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * height_mat);
  for(int y = 0; y < height_mat; y++) {
    row_pointers[y] = (png_byte*)malloc(png_get_rowbytes(png,info));
  }

  for(int y = 0; y < height_mat; y++) {
    png_bytep row = row_pointers[y];
    for(int x = 0; x < width_mat; x++) {
      png_bytep px = &(row[x * 4]);
      px[0] = mat[y][x]; 
      px[1] = mat[y][x]; 
      px[2] = mat[y][x]; 
      px[3] = 255; // No transparency 
      //printf("%4d, %4d = RGBA(%3d, %3d, %3d, %3d)\n", x, y, px[0], px[1], px[2], px[3]);
    }
  }
 
  png_write_image(png, row_pointers);
  png_write_end(png, NULL);

  for(int y = 0; y < height; y++) {
    free(row_pointers[y]);
  }
  free(row_pointers);

  fclose(fp);
}

void findWaypointCallback(const nav_msgs::Odometry& msg)
{
  ROS_INFO("Building new goal message");
 
  geometry_msgs::PoseStamped mapPose;
  mapPose.pose.position = msg.pose.pose.position;
  mapPose.pose.orientation = msg.pose.pose.orientation;
  mapPose.header = msg.header;
  geometry_msgs::PoseStamped gpsPose;

  static tf::TransformListener listener;
  listener.waitForTransform("/utm", "/map", ros::Time::now(), ros::Duration(1.0));
  try
  {

    listener.transformPose("/utm", mapPose, gpsPose);
    std::string utm_zone_tmp = "17T"; // Our UTM zone in pittsburgh

    //ROS_ERROR("x,y,z: %f,%f,%f",gpsPoint.point.x,gpsPoint.point.y,gpsPoint.point.z);

    double grudsby_lat;
    double grudsby_long;
    RobotLocalization::NavsatConversions::UTMtoLL(
        gpsPose.pose.position.y,
        gpsPose.pose.position.x,
        utm_zone_tmp,
        grudsby_lat,
        grudsby_long
    );

    //ROS_ERROR("lat: %f, lng: %f.",grudsby_lat, grudsby_long);
    double grudsby_alt = 0;

    geometry_msgs::PoseStamped goal;

    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "utm";
    goal.pose.orientation.w = 1.0;

    if (!goals.empty())
    {
      double goal_lat = goals.front().latitude;
      double goal_long = goals.front().longitude;
      
      ros::Duration time_since_goal = ros::Time::now() - last_goal_update;
      double time_duration = time_since_goal.toSec();
 
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
          double old_lat = goal_lat;
          double old_lng = goal_long;
          goal_lat = goals.front().latitude;
          goal_long = goals.front().longitude;
          time_duration = 100000; // Make sure new goals are sent immediately
          outputBorder(goal_lat, goal_long, old_lat, old_lng, resolution_, negate_, map_directory_);
          std_msgs::Bool costmap_reset; 
          map_lines_reset_pub.publish(costmap_reset);
        }
        else
        {
          last_stop_update = ros::Time::now();
          ROS_WARN("No new waypoints. Repeating previous waypoint.");
        }
      }


      if (time_duration > 1.0/goal_publish_rate) 
      {   
        last_goal_update = ros::Time::now();
        double goal_easting_x = 0;
        double goal_northing_y = 0;
        std::string utm_zone_tmp;
        //ROS_ERROR("input lat lng:%f,%f",goal_lat,goal_long);
        RobotLocalization::NavsatConversions::LLtoUTM(
            goal_lat, 
            goal_long, 
            goal_northing_y, 
            goal_easting_x, 
            utm_zone_tmp
        );

        goal.pose.position.x = goal_easting_x;
        goal.pose.position.y = goal_northing_y;
        goal.pose.position.z = grudsby_alt;
        //ROS_ERROR("UTM Goal: Northing: %f, Easting: %f, zone: %s", goal_northing_y, goal_easting_x, utm_zone_tmp.c_str());

        waypoint_pub.publish(goal);
      }
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
  catch (tf::TransformException ex)  //NOLINT
    {
      ROS_ERROR("%s", ex.what());
    }
}


void mowingPlanCallback(const grudsby_sweeping::MowingPlan& msg)
{
  goals.resize(0);
  for (grudsby_sweeping::SimpleLatLng waypoint : msg.waypoints)
  {
    Waypoint newPoint;
    newPoint.latitude = waypoint.latitude;
    newPoint.longitude = waypoint.longitude;
    newPoint.altitude = 0;
    goals.push_back(newPoint);  
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
  if (!n.getParam("grudsby_sweeping_planner/resolution", resolution_))
  {
    resolution_ = 0.05;
  }
  if (!n.getParam("grudsby_sweeping_planner/negate", negate_))
  {
    negate_ = 0;
  }
  if (!n.getParam("grudsby_sweeping_planner/map_directory", map_directory_))
  {
    map_directory_ = "/home/nvidia/GroundsBot-Software/data";
  }

  parseKMLFile();

  waypoint_pub = n.advertise<geometry_msgs::PoseStamped>("/goal", 1000);

  stop_pub = n.advertise<std_msgs::Bool>("/grudsby/stop", 100);
  
  map_lines_reset_pub = n.advertise<std_msgs::Bool>("move_base/update_map_lines", 1000);

  ros::Subscriber map_sub;
  map_sub = n.subscribe("/odometry/filtered_map", 100, findWaypointCallback);



  ros::Subscriber mowing_plan_sub;
  mowing_plan_sub = n.subscribe("/grudsby/mowing_plan", 100, mowingPlanCallback);


  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
