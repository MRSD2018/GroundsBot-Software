#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geodesy/wgs84.h>
#include <geodesy/utm.h>
#include <sstream>

ros::Publisher waypoint_pub;

class PoseDrawer
{
public:
  PoseDrawer() :
    tf2_(buffer_),  target_frame_("turtle1"),
    tf2_filter_(point_sub_, buffer_, target_frame_, 10, 0)
  {
    point_sub_.subscribe(n_, "turtle_point_stamped", 10);
    tf2_filter_.registerCallback( boost::bind(&PoseDrawer::msgCallback, this, _1) );
  }

  //  Callback to register with tf2_ros::MessageFilter to be called when transforms are available
  void msgCallback(const geometry_msgs::PointStampedConstPtr& point_ptr) 
  {
    geometry_msgs::PointStamped point_out;
    try 
    {
      buffer_.transform(*point_ptr, point_out, target_frame_);
      
      ROS_INFO("point of turtle 3 in frame of turtle 1 Position(x:%f y:%f z:%f)\n", 
             point_out.point.x,
             point_out.point.y,
             point_out.point.z);
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }
  }

private:
  std::string target_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;
  ros::NodeHandle n_;
  message_filters::Subscriber<geometry_msgs::PointStamped> point_sub_;
  tf2_ros::MessageFilter<geometry_msgs::PointStamped> tf2_filter_;

};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "waypoint_pub");
  ros::NodeHandle n;
  
  waypoint_pub = n.advertise<geometry_msgs::PoseStamped>("/goal_waypoint", 1000);

  ros::Subscriber local_odom_sub;
  //TODO: Fix topic name
  local_odom_sub = n.subscribe<nav_msgs::Odometry>("/robot/localization", 100, findWaypointCallback)

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
