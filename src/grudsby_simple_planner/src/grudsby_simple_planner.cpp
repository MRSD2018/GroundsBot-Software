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


geometry_msgs::PoseStamped goal_pose_in_odom;
nav_msgs::Odometry curr_odom;


tf::TransformListener listener;

double max_x_vel = 1;
double max_theta_vel = 4;

float Kp_lin = 1.0;
float Ki_lin = 0;
float Kd_lin = 0;
double total_lin_error = 0;
double prev_x_towards_g = 0;  

float Kp_ang = 1.0;
float Ki_ang = 0;
float Kd_ang = 0;    
double total_ang_error = 0;
double prev_theta = 0;

bool goal_set = false;

double deadband = .2;
//receive the current position
void odom_received(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  curr_odom = *odom_msg; 
}

//receive a goal position
void goal_received(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
  total_lin_error = 0;
  total_ang_error = 0;
 
  prev_x_towards_g = 0;
  prev_theta = 0;

  goal_set = true;

  geometry_msgs::PoseStamped goal_pose_in_gps;

  goal_pose_in_gps = *goal_msg;
  
  listener.transformPose("odom", goal_pose_in_gps, goal_pose_in_odom);
 
}

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
      //read goal pose
      goal_odom.pose.pose.position.x = 8;
      goal_odom.pose.pose.position.y = 0;
      goal_odom.pose.pose.position.z = 0;
   //read start pose
      curr_odom.pose.pose.position.x = 2;
      curr_odom.pose.pose.position.y = 4;
      curr_odom.pose.pose.position.z = 0;
    
      curr_odom.pose.pose.orientation.x = .146;
      curr_odom.pose.pose.orientation.y = .354;
      curr_odom.pose.pose.orientation.z = .354;
      curr_odom.pose.pose.orientation.w = .854;
    */
    if(goal_set)
    {


    
      Quaternion q(curr_odom.pose.pose.orientation.x,curr_odom.pose.pose.orientation.y,curr_odom.pose.pose.orientation.z,curr_odom.pose.pose.orientation.w);
        
      Vector3 rpy = Quaternion::ToEuler(q);
      Vector3 x_vec(cos(rpy.Z), sin(rpy.Z), 0);
      Vector3 z_vec(0, 0, 1);

 
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
        x_towards_g = cos(theta);
      }
      else
      {
        
        x_towards_g = 0;
        theta = 0;
      }
      //Bound if negative, we want GroundsBot going forwards to goal, not reverse
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

    //DEBUGGING
//      ROS_INFO("%f, %f, %f", x_vec.X, x_vec.Y, x_vec.Z);
//      ROS_INFO("%f, %f, %f", v_vec.X, v_vec.Y, v_vec.Z);
   
      ROS_INFO("%f",theta_d);
    

      ///Bound x_vel and theta_vel
      if(abs(x_vel)>max_x_vel)
      {
        x_vel = sign(x_vel)*max_x_vel;
      }
      
      if(abs(theta_vel)>max_theta_vel)
      {
        theta_vel = sign(theta_vel)*max_theta_vel;
      }
      
      ROS_INFO("%f", theta_vel);
      ROS_INFO("%f", x_vel);

      geometry_msgs::Twist msg;
      msg.linear.x = x_vel;
      msg.linear.y = 0;
      msg.linear.z = 0;
      
      msg.angular.x = 0;
      msg.angular.y = 0;
      msg.angular.z = theta_vel;
      
      velPub.publish(msg); 

    }   
// Vector2 startToGoal(goal_odom.pose.pose.position.x - curr_odom.pose.pose.position.x, goal_odom.pose.pose.position.y - curr_odom.pose.pose.position.y);

    


    //publish cmd_vel
    ros::spinOnce();
    loop_rate.sleep();
  }
}
