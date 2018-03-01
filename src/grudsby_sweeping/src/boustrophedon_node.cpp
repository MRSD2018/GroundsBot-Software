#include "boustrophedon.h"
#include "ros/console.h"
#include "ros/ros.h"
#include <string>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <curl/curl.h>
#include <fstream>
#include "navsat_conversions.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include <tf/transform_datatypes.h>
#include "winding_num.h"
#include <stdlib.h>
#include <stdio.h>
#include <png.h>
#include <vector>
static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

double implement_width_;

std::string app_url_;

bool plan_approved_;

grudsby_sweeping::MowingPlan mowing_plan_;

std::string latest_mowing_region_;

std::string pos_message_;

void odomCallback(const nav_msgs::Odometry& msg)
{
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

    double Lat;
    double Lng;
    RobotLocalization::NavsatConversions::UTMtoLL(
        gpsPose.pose.position.y, 
        gpsPose.pose.position.x,
        utm_zone_tmp,
        Lat,
        Lng
    );
    double roll, pitch, yaw;
    //ROS_ERROR("output lat lng:%f,%f",Lat,Lng);
    tf::Quaternion q(
        gpsPose.pose.orientation.x,
        gpsPose.pose.orientation.y,
        gpsPose.pose.orientation.z,
        gpsPose.pose.orientation.w
    );
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    std::stringstream ss;
    ss << "/setMowerPos?jsonData={\"lat\":";
    ss << std::setprecision(18) << std::fixed << Lat;
    ss << ",\"lng\":";
    ss << std::setprecision(18) << std::fixed << Lng;
    ss << ",\"rot\":";
    ss << std::setprecision(18) << std::fixed << -5+yaw*180.0/3.141529;
    ss << "}";
    pos_message_ = ss.str();    
  }
  catch (tf::TransformException ex) 
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

}

void outputBorder(std::string region, double resolution, double occupied_thresh, double free_thresh, double negate, double outer_edge_buffer, std::string map_directory)
{
  // Parse region
  std::vector<std::vector<double>> parsedRegion;
  ParsePath::parseLatLng(region, parsedRegion);

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
  
  max_x += 2*outer_edge_buffer;
  min_x -= 2*outer_edge_buffer;
  max_y += 2*outer_edge_buffer;
  min_y -= 2*outer_edge_buffer;      
  double width = (max_x-min_x)/resolution;
  double height = (max_y-min_y)/resolution;
  ROS_INFO("Outputting yaml");
  // Output YAML
  std::ofstream file_out((map_directory + "/map.yaml").c_str(), std::ios::binary);
  file_out << "image: map.png" << std::endl;
  file_out << "resolution: " << resolution << std::endl;
  file_out << "origin: [" << min_x << ", " << min_y << ", 0.0]" << std::endl;
  file_out << "occupied_thresh: " << occupied_thresh << std::endl;
  file_out << "free_thresh: " << free_thresh << std::endl;
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
      test_point.push_back(min_y + y*resolution);
      bool inRegion = (winding_num::wn_PnPoly(test_point, parsedRegion) != 0);
      for (int i = 0; i < parsedRegion.size()-1; i++) 
      {
        double dist = sqrt(pow((parsedRegion[i][0]-test_point[0]),2.0) + pow((parsedRegion[i][1]-test_point[1]),2.0)); 
        inRegion = inRegion || (dist < outer_edge_buffer);
      }      
       
      if (!inRegion) 
      { 
        mat[y][x] = static_cast<uint8_t>(0);
      }
    }
  }
  
  ROS_ERROR("Output the map to png");
 
  int width_mat = mat[0].size();
  int height_mat = mat.size();
 
  FILE *fp = fopen((map_directory+"/map.png").c_str(), "wb");
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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "grudsby_sweeping_planner");

  ros::NodeHandle n;

  ros::Publisher mowing_plan_pub = n.advertise<grudsby_sweeping::MowingPlan>("grudsby/mowing_plan", 1000);
 
  ros::Subscriber odom_sub;
  odom_sub = n.subscribe("/odometry/filtered_map", 100, odomCallback); 

  ros::Rate loop_rate(2);
 
  if (!n.getParam("grudsby_sweeping_planner/implement_width", implement_width_))
  {
    implement_width_ = 0.5;
  }
  if (!n.getParam("grudsby_sweeping_planner/app_url", app_url_))
  {
    app_url_ = "http://localhost:8080";
  }
  
  Boustrophedon planner(implement_width_);

  plan_approved_ = false; 
  
  int count = 0;

  while (ros::ok())
  {
    CURL *curl;
    CURLcode res;
    std::string readBuffer;

    curl = curl_easy_init();
    if(curl) {
      curl_easy_setopt(curl, CURLOPT_URL, (app_url_+"/region?polygon_id=sve").c_str());
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
      res = curl_easy_perform(curl);
      std::size_t region = readBuffer.find("regionID");
      std::size_t coordinates = readBuffer.find("coordinates");
      if ((region == std::string::npos) || (coordinates ==std::string::npos))
      {
  
      }
      else
      {
        latest_mowing_region_ = readBuffer; 
        std::string plan = "jsonData="+planner.planPath(readBuffer, mowing_plan_);
        char* toPost = new char [plan.length()+1];
        std::strcpy (toPost,plan.c_str());
        curl_easy_setopt(curl, CURLOPT_URL, (app_url_+"/savePlan").c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, toPost);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)strlen(toPost));
        res = curl_easy_perform(curl);
      }  
      CURL *curl2;
      curl2 = curl_easy_init();
      curl_easy_setopt(curl2, CURLOPT_URL, (app_url_+"/approval?polygon_id=sve").c_str());
      curl_easy_setopt(curl2, CURLOPT_WRITEFUNCTION, WriteCallback);
      curl_easy_setopt(curl2, CURLOPT_WRITEDATA, &readBuffer);
      res = curl_easy_perform(curl2);
      curl_easy_cleanup(curl2);
      std::size_t regionID = readBuffer.find("regionID");
      std::size_t approval = readBuffer.find("approval");
      if ((regionID == std::string::npos) || (approval == std::string::npos))
      {
        
      }
      else
      {
        std::string approved =readBuffer.substr(approval+12,4);
        if( !approved.compare("true") ) 
        {
          if (!plan_approved_)
          {
            // Draw the png file of the region
            outputBorder(latest_mowing_region_, 0.05, 0.65, 0.196, 1, 2, "/media/josh/Projects/GroundsBot-Software/data");
            //Send Mowing Plan to grudsby
            mowing_plan_pub.publish(mowing_plan_); 
            
          }
          plan_approved_ = true;
        }
        else
        {
          plan_approved_ = false;
        }
      }
    }
    curl_easy_cleanup(curl);
    std::string posUpdate = app_url_+pos_message_;
    //ROS_ERROR("submission: %s",posUpdate.c_str());    
    CURL *curl3;
    curl3 = curl_easy_init();
    curl_easy_setopt(curl3, CURLOPT_URL, posUpdate.c_str());
    curl_easy_setopt(curl3, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl3, CURLOPT_WRITEDATA, &readBuffer);
    res = curl_easy_perform(curl3);
    curl_easy_cleanup(curl3);
    

    ros::spinOnce();
  
    loop_rate.sleep();
    ++count;
  }
}
