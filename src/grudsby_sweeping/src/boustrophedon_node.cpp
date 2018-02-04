#include "boustrophedon.h"
#include "ros/console.h"
#include "ros/ros.h"
#include <string>
#include <iostream>
#include <curl/curl.h>


static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

double implement_width_;

std::string app_url_;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "grudsby_sweeping");

  ros::NodeHandle n;

  //ros::Publisher imu_raw_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);
 
  //ros::Subscriber arduino_sub = n.subscribe("grudsby/arduino_response", 1000, arduinisCallback);

  ros::Rate loop_rate(1);
 
  if (!n.getParam("grudsby_sweeping/implement_width", implement_width_))
  {
    implement_width_ = 0.5;
  }
  if (!n.getParam("grudsby_sweeping/app_url", app_url_))
  {
    app_url_ = "http://localhost:8080";
  }
  
  Boustrophedon planner(implement_width_);
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
        std::string plan = "jsonData="+planner.planPath(readBuffer);
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
          ROS_ERROR("approved!");
          //Send Mowing Plan
        }
      }
      
    }
    curl_easy_cleanup(curl);
    ros::spinOnce();
  
    loop_rate.sleep();
    ++count;
  }
}
