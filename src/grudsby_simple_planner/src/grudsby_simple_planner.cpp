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

geometry_msgs::PoseStamped goal_pose_in_odom;
nav_msgs::Odometry curr_odom;



double max_x_vel = 1;
double max_theta_vel = 4;

float Kp_lin = 1.0;
float Ki_lin = 0;
float Kd_lin = 0;
double total_lin_error = 0;
double prev_x_towards_g = 0;  

float Kp_ang = 7.0;
float Ki_ang = 0;
float Kd_ang = 0;    
double total_ang_error = 0;
double prev_theta = 0;

double tuner = 2; //tuning factor

bool goal_set = false;

double deadband = .2;

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
  ros::Rate loop_rate(500);

  ros::Publisher velPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  ros::Subscriber odomSub = n.subscribe("odometry/filtered", 100, odom_received);
  ros::Subscriber goalSub = n.subscribe("goal", 100, goal_received);   
    

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
      double delta_x = goal_pose_in_odom.pose.position.x - curr_odom.pose.pose.position.x;   
      double delta_y = goal_pose_in_odom.pose.position.y - curr_odom.pose.pose.position.y;   
    
      Vector3 v_vec(delta_x, delta_y, 0);

      //Find angle between vector and x direction    
      Vector3 x_cross_v = Vector3::Cross(x_vec, v_vec);
      int direction = sign(x_cross_v.Z);
      double theta = direction*Vector3::Angle(x_vec, v_vec);
      double theta_d = theta / (2*3.14159265359) * 360;
   
      //Find part of x_vec in direction of g 
      double x_towards_g;
      if (Vector3::Magnitude(v_vec)>deadband)
      {
        x_towards_g = Vector3::Magnitude(v_vec)*pow(cos(theta),tuner);
      }
      else
      {
        
        x_towards_g = 0;
        theta = 0;
      }

      //Bound if negative.  We want GroundsBot going forwards to goal, not reverse
      if (x_towards_g < 0)
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


      ///Bound x_vel and theta_vel
      if(abs(x_vel)>max_x_vel)
      {
        x_vel = sign(x_vel)*max_x_vel;
      }
      
      if(abs(theta_vel)>max_theta_vel)
      {
        theta_vel = sign(theta_vel)*max_theta_vel;
      }
      
      ROS_INFO("Angular Velocity: %f", theta_vel);
      ROS_INFO("Linear Velocity: %f", x_vel);

      //Publish /cmd_vel
      geometry_msgs::Twist msg;
      msg.linear.x = x_vel;
      msg.linear.y = 0;
      msg.linear.z = 0;
      
      msg.angular.x = 0;
      msg.angular.y = 0;
      msg.angular.z = theta_vel;
      
      velPub.publish(msg); 

    }   
    


    ros::spinOnce();
    loop_rate.sleep();
  }
}