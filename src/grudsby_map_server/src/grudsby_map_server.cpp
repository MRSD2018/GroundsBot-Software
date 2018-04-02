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
#include <ros/console.h> 
#include "grudsby_sweeping/SimpleLatLng.h"
#include "grudsby_sweeping/MowingPlan.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"
#include "tf/transform_listener.h"
#include <tf/transform_datatypes.h>
#include <png.h>
#include "Vector2.hpp"
#include <algorithm>
#include <cmath>
#include "winding_num.h"
#include <mutex>
#include <iomanip>
// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

nav_msgs::MapMetaData outline_meta_data_message_;
    
nav_msgs::GetMap::Response outline_map_resp_;

nav_msgs::MapMetaData lines_meta_data_message_;
    
nav_msgs::GetMap::Response lines_map_resp_;

double resolution_;

double outer_edge_buffer_;

std::string frame_id_;

int no_cols_;
  
int no_rows_;

double max_x_, min_x_;

double max_y_, min_y_;

ros::Publisher outline_map_pub_;

ros::Publisher outline_metadata_pub_;

ros::ServiceServer outline_service_;

ros::Publisher lines_map_pub_;

ros::Publisher lines_metadata_pub_;

ros::ServiceServer lines_service_;

std::vector<std::vector<double>> parsed_outline_;

std::string mower_region_;

std::mutex map_mutex_;

bool outlineMapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res )
{
  // request is empty; we ignore it
  // = operator is overloaded to make deep copy (tricky!)
  map_mutex_.lock(); 
  res = outline_map_resp_;
  map_mutex_.unlock(); 
  ROS_INFO("Sending map");
  return true;
}

bool linesMapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res )
{
  // request is empty; we ignore it
  // = operator is overloaded to make deep copy (tricky!)
  map_mutex_.lock(); 
  res = lines_map_resp_;
  map_mutex_.unlock();
  ROS_INFO("Sending map");
  return true;
}

bool parseMowingPlan()
{
  static tf::TransformListener listener;

  if (!listener.waitForTransform("/map", "/utm", ros::Time::now(), ros::Duration(100.0)))
  { 
    ROS_ERROR("Couldn't find utm to map transform. Mowing plan parsing failed."); 
    return false;
  }
  //ROS_ERROR("Got a map transform");
  max_x_ = -9999999999;
  max_y_ = -9999999999;
  min_x_ = 9999999999;
  min_y_ = 9999999999; 
  
  //ROS_ERROR("Size of parsed outline: %f", static_cast<double>(parsed_outline_.size())); 
  for (std::vector<double> &latlng : parsed_outline_)
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
      //ROS_ERROR("X %f Y %f",latlng[0],latlng[1]);
      max_x_ = std::max(max_x_,latlng[0]);
      min_x_ = std::min(min_x_,latlng[0]);
      max_y_ = std::max(max_y_,latlng[1]);
      min_y_ = std::min(min_y_,latlng[1]);
    
      //ROS_ERROR("MIN %f %f   MAX %f %f",min_x_,min_y_,max_x_,max_y_);
    }
    catch (tf::TransformException ex)  //NOLINT
    {
      ROS_ERROR("%s", ex.what());
    }
  }
  //ROS_ERROR("Got Transforms"); 
  // Push all edges out by "outer_edge_buffer"
  parsed_outline_.push_back(parsed_outline_[0]); 
  
  max_x_ += 2*outer_edge_buffer_;
  min_x_ -= 2*outer_edge_buffer_;
  max_y_ += 2*outer_edge_buffer_;
  min_y_ -= 2*outer_edge_buffer_;      
  double width = (max_x_-min_x_)/resolution_;
  double height = (max_y_-min_y_)/resolution_;
  
  no_cols_ = std::ceil((max_x_-min_x_)/resolution_);
  no_rows_ = std::ceil((max_y_-min_y_)/resolution_);
 
  map_mutex_.lock();
 
  outline_map_resp_.map.info.width = no_cols_; 
  outline_map_resp_.map.info.height = no_rows_;
  outline_map_resp_.map.info.resolution = resolution_;
  outline_map_resp_.map.info.origin.position.x = min_x_;
  outline_map_resp_.map.info.origin.position.y = min_y_;
  outline_map_resp_.map.info.origin.position.z = 0;
  outline_map_resp_.map.info.origin.orientation.x = 0;
  outline_map_resp_.map.info.origin.orientation.y = 0;
  outline_map_resp_.map.info.origin.orientation.z = 0;
  outline_map_resp_.map.info.origin.orientation.w = 1;
  outline_map_resp_.map.data.resize(no_cols_ * no_rows_);
  
  // for loop: j is height, i is width

  outline_map_resp_.map.info.map_load_time = ros::Time::now();
  outline_map_resp_.map.header.frame_id = frame_id_;
  outline_map_resp_.map.header.stamp = ros::Time::now();
  ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
            outline_map_resp_.map.info.width,
            outline_map_resp_.map.info.height,
            outline_map_resp_.map.info.resolution);
  outline_meta_data_message_ = outline_map_resp_.map.info;


  lines_map_resp_.map.info.width = no_cols_; 
  lines_map_resp_.map.info.height = no_rows_;
  lines_map_resp_.map.info.resolution = resolution_;
  lines_map_resp_.map.info.origin.position.x = min_x_;
  lines_map_resp_.map.info.origin.position.y = min_y_;
  lines_map_resp_.map.info.origin.position.z = 0;
  lines_map_resp_.map.info.origin.orientation.x = 0;
  lines_map_resp_.map.info.origin.orientation.y = 0;
  lines_map_resp_.map.info.origin.orientation.z = 0;
  lines_map_resp_.map.info.origin.orientation.w = 1;
  lines_map_resp_.map.data.resize(no_cols_ * no_rows_);
  
  // for loop: j is height, i is width

  lines_map_resp_.map.info.map_load_time = ros::Time::now();
  lines_map_resp_.map.header.frame_id = frame_id_;
  lines_map_resp_.map.header.stamp = ros::Time::now();
  lines_meta_data_message_ = lines_map_resp_.map.info;

  for (int x = 0; x < no_cols_; x++)
  {
    for (int y = 0; y < no_rows_; y++)
    {
      // Output PNG
      std::vector<double> test_point;
      test_point.push_back(min_x_ + x*resolution_);
      test_point.push_back(max_y_ - y*resolution_);
      bool inRegion = (winding_num::wn_PnPoly(test_point, parsed_outline_) != 0);
      for (int i = 0; i < parsed_outline_.size()-1; i++) 
      {
        double dist = sqrt(pow((parsed_outline_[i][0]-test_point[0]),2.0) + pow((parsed_outline_[i][1]-test_point[1]),2.0)); 
        inRegion = inRegion || (dist < outer_edge_buffer_);
        Vector2 fullVec;
        fullVec.X = parsed_outline_[i+1][0] - parsed_outline_[i][0];
        fullVec.Y = parsed_outline_[i+1][1] - parsed_outline_[i][1];
        Vector2 unitFull = Vector2::Normalized(fullVec);  
        Vector2 testVec;
        testVec.X = test_point[0] - parsed_outline_[i][0];
        testVec.Y = test_point[1] - parsed_outline_[i][1];
        double dotted = Vector2::Dot(testVec,unitFull);
        if ((dotted > 0) && (dotted < Vector2::Magnitude(fullVec)))
        {
          Vector2 orthVec = testVec - dotted*unitFull;
          inRegion = inRegion || (Vector2::Magnitude(orthVec) < outer_edge_buffer_);
        } 
      }             
      // write to map if in region.
      int num_to_write = 0;
      if (!inRegion) 
      { 
        num_to_write = 100;
      }
      outline_map_resp_.map.data[MAP_IDX(outline_map_resp_.map.info.width,x,outline_map_resp_.map.info.height - y - 1)] = num_to_write;
      lines_map_resp_.map.data[MAP_IDX(lines_map_resp_.map.info.width,x,lines_map_resp_.map.info.height - y - 1)] = num_to_write; // erase previous lines map
    }
  }
  map_mutex_.unlock();
  outline_metadata_pub_.publish( outline_meta_data_message_ );
  outline_map_pub_.publish( outline_map_resp_.map );
  lines_metadata_pub_.publish( lines_meta_data_message_ );
  lines_map_pub_.publish( lines_map_resp_.map );
   
  //ROS_ERROR("Updated Map Outline");
  return true;
}

void mowingPlanCallback(const grudsby_sweeping::MowingPlan& msg)
{
  //ROS_ERROR("Mowing plan callback");
  parsed_outline_.resize(0);
  std::ofstream outf(mower_region_.c_str());
  for (grudsby_sweeping::SimpleLatLng waypoint : msg.waypoints)
  {
    std::vector<double> newPoint;
    newPoint.push_back(waypoint.longitude);
    newPoint.push_back(waypoint.latitude);
    parsed_outline_.push_back(newPoint);
    outf << std::setprecision(10) << newPoint[0] << ',' << std::setprecision(10) << newPoint[1] << ',' << std::setprecision(10) << 0.0 << ' ' << std::endl;
  } 
  outf.close(); 
  //ROS_ERROR("Ready to parse mowing plan (mowingPlanCallback)");
  parseMowingPlan();
}


void parseKMLFile()
{
  std::ifstream infile(mower_region_.c_str());
  if (!infile)
  { 
    // Print an error and exit
    ROS_ERROR("Cannot open mower_path file for parsing at file location %s! No goal waypoints created!", mower_region_.c_str());
    return; 
  }
  parsed_outline_.resize(0);
  std::string line;
  while (std::getline(infile, line))
  {
    ROS_INFO("Parsing mower_region file to get waypoints.");
    std::stringstream ss;
    ss << line;
    char ch;
    ss >> ch; 
    if (ch == ('<'))
    { 
      continue;
    }
    
    ss.putback(ch);
    
    char count = 0;
    std::string combined_coords;
    std::stringstream combined_ss;
    std::getline(ss, combined_coords, ' ');
    combined_ss << combined_coords;
    std::vector<double> new_point;     
    std::string s_coord;
    while (std::getline(combined_ss, s_coord, ','))
    {
      new_point.push_back(atof(s_coord.c_str()));
      //ROS_ERROR("parsing: %s", s_coord.c_str());
    }
    parsed_outline_.push_back(new_point);
  }
  infile.close(); 
  //ROS_ERROR("Ready to parse mowing plan (parseKMLFile).");
  parseMowingPlan();
}

void waypointLinesCallback(const grudsby_sweeping::MowingPlan& msg)
{
  //ROS_ERROR("Waypoint lines callback");
  std::vector<std::vector<double>> parsedLines;
  for (grudsby_sweeping::SimpleLatLng waypoint : msg.waypoints)
  {
    std::vector<double> newPoint;
    newPoint.push_back(waypoint.longitude);
    newPoint.push_back(waypoint.latitude);
    parsedLines.push_back(newPoint);
  } 

  static tf::TransformListener listener;

  if (!listener.waitForTransform("/map", "/utm", ros::Time::now(), ros::Duration(1.0)))
  { 
    return;
  }
  for (std::vector<double> &latlng : parsedLines)
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
    }
    catch (tf::TransformException ex)  //NOLINT
    {
      ROS_ERROR("%s", ex.what());
    }
  }
  ROS_INFO("Got Transforms"); 
  // Push all edges out by "outer_edge_buffer"
  parsedLines.push_back(parsedLines[0]);

  for (int x = 0; x < no_cols_; x++)
  {
    for (int y = 0; y < no_rows_; y++)
    {
      std::vector<double> test_point;
      test_point.push_back(min_x_ + x*resolution_);
      test_point.push_back(max_y_ - y*resolution_);
      double minDist = 99999999999;
      for (int i = 0; i < parsedLines.size()-1; i++) 
      {
        double dist = sqrt(pow((parsedLines[i][0]-test_point[0]),2.0) + pow((parsedLines[i][1]-test_point[1]),2.0)); 
        minDist = std::min(minDist, dist);
        Vector2 fullVec;
        fullVec.X = parsedLines[i+1][0] - parsedLines[i][0];
        fullVec.Y = parsedLines[i+1][1] - parsedLines[i][1];
        Vector2 unitFull = Vector2::Normalized(fullVec);  
        Vector2 testVec;
        testVec.X = test_point[0] - parsedLines[i][0];
        testVec.Y = test_point[1] - parsedLines[i][1];
        double dotted = Vector2::Dot(testVec,unitFull);
        if ((dotted > 0) && (dotted < Vector2::Magnitude(fullVec)))
        {
          Vector2 orthVec = testVec - dotted*unitFull;
          minDist = std::min(minDist, Vector2::Magnitude(orthVec));
        } 
      }      
      double mapped = std::min(pow(minDist,0.5)*5.0,5.0)*5.0;

      bool inRegion = (winding_num::wn_PnPoly(test_point, parsed_outline_) != 0);
      for (int i = 0; i < parsed_outline_.size()-1; i++) 
      {
        double dist = sqrt(pow((parsed_outline_[i][0]-test_point[0]),2.0) + pow((parsed_outline_[i][1]-test_point[1]),2.0)); 
        inRegion = inRegion || (dist < outer_edge_buffer_);
        Vector2 fullVec;
        fullVec.X = parsed_outline_[i+1][0] - parsed_outline_[i][0];
        fullVec.Y = parsed_outline_[i+1][1] - parsed_outline_[i][1];
        Vector2 unitFull = Vector2::Normalized(fullVec);  
        Vector2 testVec;
        testVec.X = test_point[0] - parsed_outline_[i][0];
        testVec.Y = test_point[1] - parsed_outline_[i][1];
        double dotted = Vector2::Dot(testVec,unitFull);
        if ((dotted > 0) && (dotted < Vector2::Magnitude(fullVec)))
        {
          Vector2 orthVec = testVec - dotted*unitFull;
          inRegion = inRegion || (Vector2::Magnitude(orthVec) < outer_edge_buffer_);
        } 
      }             
      // write to map if in region.
      
      if (!inRegion) 
      { 
        mapped = 100;
      }
      map_mutex_.lock(); 
      lines_map_resp_.map.data[MAP_IDX(lines_map_resp_.map.info.width,x,lines_map_resp_.map.info.height - y - 1)] = mapped; // write new lines map
      map_mutex_.unlock();
    }
  }
  lines_metadata_pub_.publish( lines_meta_data_message_ );
  lines_map_pub_.publish( lines_map_resp_.map );
  
  //ROS_ERROR("Updated Lines");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grudsby_map_server");
  ros::NodeHandle n;

  if (!n.getParam("grudsby_map_server/resolution", resolution_))
  {
    resolution_ = 0.05;
  }
  if (!n.getParam("grudsby_map_server/outer_edge_buffer", outer_edge_buffer_))
  {
    outer_edge_buffer_ = 1;
  }
  if (!n.getParam("grudsby_map_server/frame_id", frame_id_))
  {
    frame_id_ = "map";
  }
  if (!n.getParam("grudsby_waypoint/mower_region_file", mower_region_))
  {
    mower_region_ = "/home/nvidia/GroundsBot-Software/data/mower_region.kml";
  }
 

  ros::Subscriber mowing_plan_sub;
  mowing_plan_sub = n.subscribe("/grudsby/mowing_region", 100, mowingPlanCallback);

  ros::Subscriber waypoint_lines_sub;
  waypoint_lines_sub = n.subscribe("grudsby/waypoint_lines", 100, waypointLinesCallback);

  // Service for outline
  outline_service_ = n.advertiseService("static_map", &outlineMapCallback);
  
  // Latched publisher for metadata
  outline_metadata_pub_ = n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  outline_metadata_pub_.publish( outline_meta_data_message_ );

  // Latched publisher for data
  outline_map_pub_ = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  outline_map_pub_.publish( outline_map_resp_.map );

  // Service for waypoint lines
  lines_service_ = n.advertiseService("static_map_lines", &linesMapCallback);
  
  // Latched publisher for metadata
  lines_metadata_pub_ = n.advertise<nav_msgs::MapMetaData>("map_metadata_lines", 1, true);
  lines_metadata_pub_.publish( lines_meta_data_message_ );

  // Latched publisher for data
  lines_map_pub_ = n.advertise<nav_msgs::OccupancyGrid>("map_lines", 1, true);
  lines_map_pub_.publish( lines_map_resp_.map );

  parseKMLFile();

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
