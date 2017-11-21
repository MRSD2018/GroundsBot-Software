<<<<<<< HEAD
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include <sstream>
#include <stdlib.h>
#include <math.h>
#include "Vector3.hpp"
#include "Quaternion.hpp"
#include "Matrix3x3.hpp"

// Adjust these update rates to correspond with the sensor data rates we plan to use - note, all are close to 1-2 per second currently for ease of visualization

double poseUpdateRate = 0.002; // delay between updates in seconds
double imuUpdateRate = 0.01; // delay between imu updates
double gpsUpdateRate =  0.8; // delay between gps updates
double odomUpdateRate = 0.02; // delay between odometry updates

// This timeout is to stop executing a cmd_vel command after not hearing any new ones for a certain period of time

double maxCmdVelTimeout = 10.0;
ros::Time lastCmdVelTime;
geometry_msgs::Twist lastCmdVel;

// Used to estimate the IMU data and track positions and orientations
Matrix3x3 lastOrientation = Matrix3x3::Identity();
Matrix3x3 lastImuOrientation = Matrix3x3::Identity();
Vector3 lastPosition(0,0,0);
Vector3 lastVelocity(0,0,0);
Vector3 lastAngularVel(0,0,0);

Vector3 lastImuPosition(0,0,0);
Vector3 lastImuVelocity(0,0,0);

Vector3 lastGpsPosition(0,0,0);

// Basic position and rotation of the robot. These x and y positions are mapped onto a "hilly" terrain to simulate 3d motions.
typedef struct {
  double x;   // x position in meters
  double y;   // y position in meters
  double yaw; // Rotation in Radians
} simplePose;

simplePose grudsbyPose; // The current pose of the robot.

// Generate a normally distributed random number
double rand_normal(double mean, double stddev)
{//Box muller method
    static double n2 = 0.0;
    static int n2_cached = 0;
    if (!n2_cached)
    {
        double x, y, r;
        do
        {
            x = 2.0*rand()/RAND_MAX - 1;
            y = 2.0*rand()/RAND_MAX - 1;

            r = x*x + y*y;
        }
        while (r == 0.0 || r > 1.0);
        {
            double d = sqrt(-2.0*log(r)/r);
            double n1 = x*d;
            n2 = y*d;
            double result = n1*stddev + mean;
            n2_cached = 1;
            return result;
        }
    }
    else
    {
        n2_cached = 0;
        return n2*stddev + mean;
    }
}

// Update the cmd_vel values.
void wheelVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  lastCmdVelTime = ros::Time::now();
  lastCmdVel = *msg;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "grudsby_sim_node");

  ros::NodeHandle n;

  ros::Publisher gpsPub = n.advertise<sensor_msgs::NavSatFix>("fix", 1000);

  ros::Publisher imuPub = n.advertise<sensor_msgs::Imu>("imu/data", 1000);

  ros::Publisher encoderPub = n.advertise<nav_msgs::Odometry>("grudsby/odometry", 1000);

  ros::Publisher truePosePub = n.advertise<nav_msgs::Odometry>("grudsby/trueOdometry", 1000);

  ros::Subscriber wheelSub = n.subscribe("cmd_vel", 1000, wheelVelCallback);

  grudsbyPose.x = 0;
  grudsbyPose.y = 0;
  grudsbyPose.yaw = 0;

  lastCmdVelTime = ros::Time::now();
  lastCmdVel.linear.x = 0;
  lastCmdVel.linear.y = 0;
  lastCmdVel.linear.z = 0;
  lastCmdVel.angular.x = 0;
  lastCmdVel.angular.y = 0;
  lastCmdVel.angular.z = 0;

  ros::Rate loop_rate(4000);
  ros::Time lastPoseUpdate = ros::Time::now();
  
  ros::Time lastImuUpdate = ros::Time::now();
  
  ros::Time lastGpsUpdate = ros::Time::now();
  
  ros::Time lastOdomUpdate = ros::Time::now();

  int count = 0;  
  while (ros::ok())
  {
    // Execute the pose update code at a certain rate
    if ((ros::Time::now() - lastPoseUpdate).toSec() > poseUpdateRate) {
      lastPoseUpdate = ros::Time::now();
      if ((ros::Time::now() - lastCmdVelTime).toSec() < maxCmdVelTimeout) {
        //Update the pose
        grudsbyPose.yaw += lastCmdVel.angular.z * poseUpdateRate/2.0 * rand_normal(1, 0.03);
        grudsbyPose.x += lastCmdVel.linear.x * poseUpdateRate * cos(grudsbyPose.yaw) * rand_normal(1, 0.02);
        grudsbyPose.y += lastCmdVel.linear.x * poseUpdateRate * sin(grudsbyPose.yaw) * rand_normal(1, 0.02);
        grudsbyPose.yaw += lastCmdVel.angular.z * poseUpdateRate/2.0 * rand_normal(1, 0.03);
      } 

      Vector3 vecAlongSlope = Vector3(1, 0, -1 * 3.1415 / 4.0 * sin(fmod(grudsbyPose.x,8) / 4.0 * 3.1415));
      Vector3 vecZ = Vector3::Cross( Vector3::Normalized(vecAlongSlope),Vector3(0,1,0));
      Matrix3x3 rotAroundZ = Matrix3x3::FromQuaternion(Quaternion::FromAngleAxis(grudsbyPose.yaw, vecZ));
      Vector3 vecY = rotAroundZ * Vector3(0,1,0);
      Vector3 vecX = Vector3::Cross(vecY, vecZ);


      //Vector3 yawVec = Vector3(cos(grudsbyPose.yaw), sin(grudsbyPose.yaw), 0);
      //Vector3 vecY = Vector3::Normalized(Vector3::Cross(vecZ, yawVec));
      //Vector3 vecX = Vector3::Cross(vecY, vecZ);

      Matrix3x3 newOrientation = Matrix3x3::Transpose(Matrix3x3(vecX, vecY, vecZ));

      Quaternion deltaOrientation = Matrix3x3::ToQuaternion(Matrix3x3::Transpose(lastOrientation) * newOrientation);
      double angle;
      Vector3 axis;
      Quaternion::ToAngleAxis(deltaOrientation, angle, axis);
      double angularVel = angle/poseUpdateRate;
      lastAngularVel = angularVel*axis;
      Vector3 newPosition = Vector3(grudsbyPose.x, grudsbyPose.y, 1 * cos(fmod(grudsbyPose.x,8) / 4.0 * 3.1415 ));

      Vector3 deltaPosition = newPosition - lastPosition;

      Vector3 newVelocity = deltaPosition / poseUpdateRate;

      Vector3 accel = (newVelocity - lastVelocity) / poseUpdateRate;

      lastVelocity = newVelocity;

      lastPosition = newPosition;

      lastOrientation = newOrientation;
    }
  
    // Execute the gps update code at a certain rate
    if ((ros::Time::now() - lastGpsUpdate).toSec() > gpsUpdateRate) {
      Vector3 GpsPosition = lastPosition + Matrix3x3::Transpose(lastOrientation) * Vector3(0.2475, 0.0, 0.2231);
      sensor_msgs::NavSatFix gps;
      gps.header.stamp = ros::Time::now();
      lastGpsUpdate = ros::Time::now();
      gps.header.frame_id = "gps";
      gps.status.status = gps.status.STATUS_SBAS_FIX;
      gps.status.service = gps.status.SERVICE_GPS;
     
     // gps.longitude = (GpsPosition.X + rand_normal(0, .01)) / 6371393.0 * 180.0 / 3.1415;         
     // gps.latitude = (GpsPosition.Y + rand_normal(0, .01)) / 6371393.0 * 180.0 / 3.1415;  
     // gps.altitude = rand_normal(0, .015) + GpsPosition.Z;
      
      gps.longitude = (GpsPosition.X ) / 6378137.0 * 180.0 / 3.1415265359 + 10;         
      gps.latitude = (GpsPosition.Y ) / 6378137.0 * 180.0 / 3.1415265359 + 10;  
      gps.altitude = GpsPosition.Z;
      
      gps.position_covariance[0] = 0; // NOTE: NEED TO ADJUST COVARIANCES
      gps.position_covariance[4] = 0; // NOTE: NEED TO ADJUST COVARIANCES
      gps.position_covariance[8] = 0; // NOTE: NEED TO ADJUST COVARIANCES
      
     // gps.position_covariance[0] = 0.02; // NOTE: NEED TO ADJUST COVARIANCES
     // gps.position_covariance[4] = 0.02; // NOTE: NEED TO ADJUST COVARIANCES
     // gps.position_covariance[8] = 0.05; // NOTE: NEED TO ADJUST COVARIANCES
      gps.position_covariance_type = gps.COVARIANCE_TYPE_APPROXIMATED;
      gpsPub.publish(gps);
      lastGpsPosition = GpsPosition;
    }

    // Execute the imu update code at a certain rate
    if ((ros::Time::now() - lastImuUpdate).toSec() > imuUpdateRate) {
  
      
      Matrix3x3 newImuOrientation = Matrix3x3::FromQuaternion(Quaternion(0, 0,0,1)) * lastOrientation;
      
      Quaternion deltaOrientation = Matrix3x3::ToQuaternion(Matrix3x3::Transpose(lastImuOrientation) * newImuOrientation);
      double angle;
      Vector3 axis;
      Quaternion::ToAngleAxis(deltaOrientation, angle, axis);
      double angularVel = angle/imuUpdateRate;

      Vector3 newImuPosition = lastPosition + lastOrientation * Vector3(0.2351, -0.0687, 0.1919);

      Vector3 deltaPosition = Matrix3x3::Transpose(newImuOrientation) * (newImuPosition - lastImuPosition);

      Vector3 newImuVelocity = deltaPosition / imuUpdateRate;

      Vector3 accel = (newImuVelocity - lastImuVelocity) / imuUpdateRate;

      lastImuVelocity = newImuVelocity;

      lastImuPosition = newImuPosition;

      lastImuOrientation = newImuOrientation;

      Vector3 noiseAxis = Vector3::Normalized(Vector3(rand_normal(0, 1), rand_normal(0, 1), rand_normal(0, 1)));
      double angularNoise = rand_normal(0,0.001);
      Matrix3x3 noiseMat = Matrix3x3::FromQuaternion(Quaternion::FromAngleAxis(angularNoise, noiseAxis));


      Quaternion ori = Matrix3x3::ToQuaternion(noiseMat*newImuOrientation);
      
      
      sensor_msgs::Imu imu;
      imu.header.stamp = ros::Time::now();
      lastImuUpdate = ros::Time::now();
      imu.header.frame_id = "imu_link";
      imu.orientation.x = ori.X;
      imu.orientation.y = ori.Y;
      imu.orientation.z = ori.Z;
      imu.orientation.w = ori.W;
      imu.orientation_covariance[0] = 0.01; // NOTE: NEED TO ADJUST COVARIANCES
      imu.orientation_covariance[4] = 0.01; // NOTE: NEED TO ADJUST COVARIANCES
      imu.orientation_covariance[8] = 0.01; // NOTE: NEED TO ADJUST COVARIANCES
      imu.angular_velocity.x = angularVel*axis.X + rand_normal(0, .005);
      imu.angular_velocity.y = angularVel*axis.Y + rand_normal(0, .005);
      imu.angular_velocity.z = angularVel*axis.Z + rand_normal(0, .005);
      imu.angular_velocity_covariance[0] = 0.01; // NOTE: NEED TO ADJUST COVARIANCES
      imu.angular_velocity_covariance[4] = 0.01; // NOTE: NEED TO ADJUST COVARIANCES
      imu.angular_velocity_covariance[8] = 0.01; // NOTE: NEED TO ADJUST COVARIANCES
      imu.linear_acceleration.x = accel.X + rand_normal(0, .015);
      imu.linear_acceleration.y = accel.Y + rand_normal(0, .015);
      imu.linear_acceleration.z = accel.Z + rand_normal(0, .015);
      imu.linear_acceleration_covariance[0] = 0.2; // NOTE: NEED TO ADJUST COVARIANCES
      imu.linear_acceleration_covariance[4] = 0.2; // NOTE: NEED TO ADJUST COVARIANCES
      imu.linear_acceleration_covariance[8] = 0.2; // NOTE: NEED TO ADJUST COVARIANCES
      imuPub.publish(imu);
    }


    // Execute the odom update code at a certain rate
    if ((ros::Time::now() - lastOdomUpdate).toSec() > odomUpdateRate) {
      Vector3 noiseAxis = Vector3::Normalized(Vector3(rand_normal(0, 1), rand_normal(0, 1), rand_normal(0, 1)));
      double angularNoise = rand_normal(0,0.001);
      Matrix3x3 noiseMat = Matrix3x3::FromQuaternion(Quaternion::FromAngleAxis(angularNoise, noiseAxis));

      Quaternion lastOri = Matrix3x3::ToQuaternion(noiseMat * lastOrientation);


      nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time::now();
      lastOdomUpdate = ros::Time::now();
      odom.header.frame_id = "base_link";
      odom.child_frame_id = "base_link";
      odom.pose.pose.orientation.x = 0;//lastOri.X;
      odom.pose.pose.orientation.y = 0;//lastOri.Y;
      odom.pose.pose.orientation.z = 0;//lastOri.Z;
      odom.pose.pose.orientation.w = 1;//lastOri.W;
      odom.pose.pose.position.x = 0;//lastPosition.X + rand_normal(0, .01);
      odom.pose.pose.position.y = 0;//lastPosition.Y + rand_normal(0, .01);
      odom.pose.pose.position.z = 0;//lastPosition.Z + rand_normal(0, .01);
      odom.pose.covariance[0] = 0.001; // NOTE: NEED TO ADJUST COVARIANCES
      odom.pose.covariance[7] = 0.001; // NOTE: NEED TO ADJUST COVARIANCES
      odom.pose.covariance[14] = 0.001; // NOTE: NEED TO ADJUST COVARIANCES
      odom.pose.covariance[21] = 0.02; // NOTE: NEED TO ADJUST COVARIANCES
      odom.pose.covariance[28] = 0.02; // NOTE: NEED TO ADJUST COVARIANCES
      odom.pose.covariance[35] = 0.02; // NOTE: NEED TO ADJUST COVARIANCES
      
      


      odom.twist.twist.angular.z = (Matrix3x3::Transpose(lastOrientation) * lastAngularVel).Z + rand_normal(0, .001);
      odom.twist.twist.linear.x = (Matrix3x3::Transpose(lastOrientation) * lastVelocity).X + rand_normal(0, .001);


      odom.twist.covariance[0] = 0.001; // NOTE: NEED TO ADJUST COVARIANCES
      odom.twist.covariance[7] = 0.001; // NOTE: NEED TO ADJUST COVARIANCES
      odom.twist.covariance[14] = 0.001; // NOTE: NEED TO ADJUST COVARIANCES
      odom.twist.covariance[21] = 0.001; // NOTE: NEED TO ADJUST COVARIANCES
      odom.twist.covariance[28] = 0.001; // NOTE: NEED TO ADJUST COVARIANCES
      odom.twist.covariance[35] = 0.001; // NOTE: NEED TO ADJUST COVARIANCES
      
      encoderPub.publish(odom);


      Quaternion trueOri = Matrix3x3::ToQuaternion(lastOrientation);
      odom.header.frame_id = "odom";
      odom.child_frame_id = "odom";
      
      odom.pose.pose.orientation.x = trueOri.X;
      odom.pose.pose.orientation.y = trueOri.Y;
      odom.pose.pose.orientation.z = trueOri.Z;
      odom.pose.pose.orientation.w = trueOri.W;
      odom.pose.pose.position.x = lastPosition.X;
      odom.pose.pose.position.y = lastPosition.Y;
      odom.pose.pose.position.z = lastPosition.Z;

      truePosePub.publish(odom);
    }

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}

