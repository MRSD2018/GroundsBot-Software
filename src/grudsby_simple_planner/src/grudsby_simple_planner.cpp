#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "Vector3.hpp"
#include "Quaternion.hpp"
#include "Matrix3x3.hpp"
#include "Vector2.hpp"
#include "tf/transform_listener.h"
#include "math.h"
#include <grudsby_simple_planner/SimplePlannerDebug.h>
#include "ros/console.h"

geometry_msgs::PoseStamped goal_pose_in_odom;
nav_msgs::Odometry curr_odom;

bool wait_at_waypoint;
ros::Time last_waypoint_update;
double prev_goal_x = 0;
double prev_goal_y = 0;

double max_x_vel = 1;
double max_theta_vel = 1;
double max_vel_delta = 0.05;
double max_theta_vel_delta = 0.05;
double prev_x_vel = 0;
double prev_theta_vel = 0;

//set in params
float Kp_lin;
float Ki_lin;
float Kd_lin;
double total_lin_error = 0;
double prev_x_towards_g = 0;

float Kp_ang;
float Ki_ang;
float Kd_ang;   
double total_ang_error = 0;
double prev_theta = 0;
double goal_noise;
double tuner = 2; //tuning factor

bool goal_set = false;

double deadband = .1;

/*
  odom_received: Callback function called when msg received on /odometry/filtered topic. 
  Receives a goal odom broadcasted by grudsby_localization
  
  @param geometry_msgs/PoseStamped odom_msg: the odom pose being broadcasted
*/
void odom_received(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  curr_odom = *odom_msg; 
}

/*
  goal_received: Callback function called when msg received on /goal topic. 
  Receives a goal pose broadcasted by grudsby_waypoints
  
  @param geometry_msgs/PoseStamped goal_msg: the goal pose being broadcasted
*/
void goal_received(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
  
  static tf::TransformListener listener;
  total_lin_error = 0;
  total_ang_error = 0;
 
  prev_x_towards_g = 0;
  prev_theta = 0;

  goal_set = true;

  geometry_msgs::PoseStamped goal_pose_in_gps;

  goal_pose_in_gps = *goal_msg;


  listener.waitForTransform("/odom","/utm",ros::Time::now(),ros::Duration(1.0));   
  try
  {  
    listener.transformPose("/odom", goal_pose_in_gps, goal_pose_in_odom);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
    goal_set = false;
  }
}

/*
  sign: homemade sign function that returns the sign of a given double

  @param double d: Double you want sign of
  
  @return: -1 if d<0, 1 if d>=0
*/

int sign(double d)
{
  if (d < 0)
  {
    return -1;
  }
  else
  {
    return 1;
  }
}



int main(int argc, char **argv) {

  ros::init(argc, argv, "grudsby_simple_planner");

  ros::NodeHandle n;
  ros::Rate loop_rate(50);

//   float Kp_lin = .7;
// float Ki_lin = 0;
// float Kd_lin = 0;
// double total_lin_error = 0;
// double prev_x_towards_g = 0;  

// float Kp_ang = 4;
// float Ki_ang = 0;
// float Kd_ang = 6;    

  //get params and set defaults if no param
  if (!n.getParam("grudsby_simple_planner/sp_kp_lin", Kp_lin))
    Kp_lin = .7;
  if (!n.getParam("grudsby_simple_planner/sp_ki_lin", Ki_lin))
    Ki_lin = 0;
  if (!n.getParam("grudsby_simple_planner/sp_kd_lin", Ki_lin))
    Kd_lin = 0;
  if (!n.getParam("grudsby_simple_planner/sp_kp_ang", Kp_ang))
    Kp_ang = 4;
  if (!n.getParam("grudsby_simple_planner/sp_ki_ang", Ki_ang))
    Ki_ang = 0;
  if (!n.getParam("grudsby_simple_planner/sp_kd_ang", Kd_ang))
    Kd_ang = 0;
  if (!n.getParam ("grudsby_simple_planner/wait_at_waypoint", wait_at_waypoint))
    wait_at_waypoint = true;
  if (!n.getParam ("grudsby_simple_planner/goal_noise", goal_noise))
    goal_noise = 1.0;

  ros::Publisher velPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  ros::Subscriber odomSub = n.subscribe("odometry/filtered", 100, odom_received);
  ros::Subscriber goalSub = n.subscribe("goal", 100, goal_received);   
    
  ros::Publisher debugPub = n.advertise<grudsby_simple_planner::SimplePlannerDebug>("/grudsby/debug/simplePlannerDebug", 100);

  while (ros::ok())
  {
    //DEBUGGING
    /*   
      goal_set = true;
      //goal pose
      goal_odom.pose.pose.position.x = 8;
      goal_odom.pose.pose.position.y = 0;
      goal_odom.pose.pose.position.z = 0;
      //start pose
      curr_odom.pose.pose.position.x = 2;
      curr_odom.pose.pose.position.y = 4;
      curr_odom.pose.pose.position.z = 0;
    
      curr_odom.pose.pose.orientation.x = .146;
      curr_odom.pose.pose.orientation.y = .354;
      curr_odom.pose.pose.orientation.z = .354;
      curr_odom.pose.pose.orientation.w = .854;
    */

    //Don't try to navigate unless a goal has been broadcasted
    if(goal_set)
    {


      //Determine current heading
      Quaternion q(curr_odom.pose.pose.orientation.x,curr_odom.pose.pose.orientation.y,curr_odom.pose.pose.orientation.z,curr_odom.pose.pose.orientation.w);
        
      Vector3 rpy = Quaternion::ToEuler(q);
     
      Vector3 x_vec(cos(rpy.Z), sin(rpy.Z), 0);

      //Find vector between start and goal
      double current_goal_x = goal_pose_in_odom.pose.position.x;
      double current_goal_y = goal_pose_in_odom.pose.position.y;
      double delta_x = current_goal_x - curr_odom.pose.pose.position.x;   
      double delta_y = current_goal_y - curr_odom.pose.pose.position.y;   
    
      Vector3 v_vec(delta_x, delta_y, 0);
      
      //Find angle between vector and x direction    
      Vector3 x_cross_v = Vector3::Cross(x_vec, v_vec);
      int direction = sign(x_cross_v.Z);
      double theta =  direction*Vector3::Angle(x_vec, v_vec);
      double theta_d = theta / (2*3.14159265359) * 360;
   
            /*if (Vector3::Magnitude(v_vec)<deadband)
      {
         x_towards_g = 0;
         theta = 0;
     
      }*/
      
      //Find part of x_vec in direction of g 
      double x_towards_g;
      

      x_towards_g = Vector3::Magnitude(v_vec)*pow(cos(theta),tuner);
      
      //Bound if negative.  We want GroundsBot going forwards to goal, not reverse
      if (cos(theta) < 0)
      {
        x_towards_g = 0;
      }

    
      //Use PID controller to set speed of x_vel and theta_vel
      //Calculate total error and delta error
      total_lin_error = total_lin_error + x_towards_g;
      double delta_lin_error = prev_x_towards_g - x_towards_g;
      prev_x_towards_g = x_towards_g;

      total_ang_error = total_ang_error + theta;
      double delta_ang_error = prev_theta - theta;
      prev_theta = theta;

      double x_vel = Kp_lin*x_towards_g + Ki_lin*total_lin_error + Kd_lin*delta_lin_error;
      double theta_vel = Kp_ang*theta + Ki_ang*total_ang_error + Kd_ang*delta_ang_error;


      
      double delta_x_vel = x_vel - prev_x_vel;
      double delta_theta_vel = theta_vel - prev_theta_vel;

      if ( fabs(delta_x_vel) > max_vel_delta )
      {
        x_vel = prev_x_vel + sign(delta_x_vel)*max_vel_delta;
      }
      if ( fabs(delta_theta_vel) > max_theta_vel_delta )
      {
        theta_vel = prev_theta_vel + sign(delta_theta_vel)*max_theta_vel_delta;
      }
      

      //If we've received a new goal and wait_at_waypoint param is set
      //publish 0 velocity for 10 secs
      //Assuming new goal is more than 0.3 meters away in any direction
      if ( ((current_goal_x > (prev_goal_x + goal_noise)) || (current_goal_x < (prev_goal_x - goal_noise))) ||
            ((current_goal_y > (prev_goal_y + goal_noise)) || (current_goal_y < (prev_goal_y - goal_noise))) )
      {
        ROS_INFO("New Waypoint found.");
        last_waypoint_update = ros::Time::now();
        prev_goal_x = current_goal_x;
        prev_goal_y = current_goal_y;
      }
      
      ros::Duration wait = ros::Time::now() - last_waypoint_update;
      if( wait_at_waypoint && (wait.toSec() < 10.0) )
      {
        ROS_INFO("Waiting at waypoint.");
        x_vel = 0;
        theta_vel = 0;
      }
      
      ///Bound x_vel and theta_vel
      double x_vel_bound=x_vel;
      double theta_vel_bound=theta_vel;

      if(fabs(x_vel)>=max_x_vel)
      {
        x_vel_bound = sign(x_vel)*max_x_vel;
      } 
      prev_x_vel = x_vel_bound;
      
      if(fabs(theta_vel)>=max_theta_vel)
      {
        theta_vel_bound = sign(theta_vel)*max_theta_vel;
      } 
      prev_theta_vel = theta_vel_bound;
      //Publish /cmd_vel
      geometry_msgs::Twist msg;
      msg.linear.x = x_vel_bound;
      msg.linear.y = 0;
      msg.linear.z = 0;
      
      msg.angular.x = 0;
      msg.angular.y = 0;
      msg.angular.z = theta_vel_bound;
      
      velPub.publish(msg); 

      //Publish debugs
      grudsby_simple_planner::SimplePlannerDebug debug_msg;
      debug_msg.theta_d = theta_d;
      debug_msg.theta_r = theta;
      debug_msg.x_towards_g = x_towards_g;
      debug_msg.delta_x = delta_x;
      debug_msg.delta_y = delta_y;
      debug_msg.goalx = goal_pose_in_odom.pose.position.x;
      debug_msg.goaly = goal_pose_in_odom.pose.position.y;

      debugPub.publish(debug_msg);
    }   
    


    ros::spinOnce();
    loop_rate.sleep();
  }
}
