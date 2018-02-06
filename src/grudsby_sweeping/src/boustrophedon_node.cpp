#include "boustrophedon.h"
#include "ros/console.h"
#include "ros/ros.h"
#include <string>
#include <iostream>
#include <curl/curl.h>
#include "navsat_conversions.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include <tf/transform_datatypes.h>

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

double implement_width_;

std::string app_url_;

bool plan_approved_;

grudsby_sweeping::MowingPlan mowing_plan_;

std::string pos_message_;

void odomCallback(const nav_msgs::Odometry& msg)
{
  geometry_msgs::PoseStamped mapPose;
  mapPose.pose.position = msg.pose.pose.position;
  
  geometry_msgs::PoseStamped gpsPose;
  static tf::TransformListener listener;
  listener.waitForTransform("/utm", "/map", ros::Time::now(), ros::Duration(1.0));
  try
  {
    
    listener.transformPose("/utm", mapPose, gpsPose);
    std::string utm_zone_tmp;
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
    tf::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    );
    
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    std::stringstream ss;
    ss << "/setMowerPos?jsonData={\"lat\":";
    ss << std::setprecision(18) << std::fixed << Lat;
    ss << ",\"lng\":";
    ss << std::setprecision(18) << std::fixed << Lng;
    ss << ",\"rot\":";
    ss << std::setprecision(18) << std::fixed << 90-yaw;
    ss << "}" << std::endl;
    pos_message_ = ss.str();    
  }
  catch (tf::TransformException ex) 
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grudsby_sweeping_planner");

  ros::NodeHandle n;

  ros::Publisher mowing_plan_pub = n.advertise<grudsby_sweeping::MowingPlan>("grudsby/mowing_plan", 1000);
 
  ros::Subscriber odom_sub;
  odom_sub = n.subscribe("/odometry/filtered_map", 100, odomCallback); 

  ros::Rate loop_rate(10);
 
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
            //Send Mowing Plan
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
    
    CURL *curl3;
    curl3 = curl_easy_init();
    curl_easy_setopt(curl3, CURLOPT_URL, (app_url_+pos_message_).c_str());
    curl_easy_setopt(curl3, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl3, CURLOPT_WRITEDATA, &readBuffer);
    res = curl_easy_perform(curl3);
    curl_easy_cleanup(curl3);


    ros::spinOnce();
  
    loop_rate.sleep();
    ++count;
  }
}
