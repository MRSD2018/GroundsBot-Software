#include "OdomHeading.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/MagneticField.h"
#include "grudsby_lowlevel/ArduinoResponse.h"


OdomHeading odom_tracker_ = OdomHeading();

ros::Publisher mag_pub_;

void imuMagCallback(const sensor_msgs::MagneticField::ConstPtr& mag_msg)
{
  sensor_msgs::MagneticField mag_out;
  mag_out.header = mag_msg->header;
  mag_out.magnetic_field_covariance = mag_msg->magnetic_field_covariance;
  Vector3 field = odom_tracker_.processMagMessage(Vector3(mag_msg->magnetic_field.x, 
                                                          mag_msg->magnetic_field.y,
                                                          mag_msg->magnetic_field.z));
  mag_out.magnetic_field.x = field.X;
  mag_out.magnetic_field.y = field.Y;
  mag_out.magnetic_field.z = field.Z;
  mag_pub_.publish(mag_out);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  odom_tracker_.updateIMU(Quaternion(imu_msg->orientation.x,
                                imu_msg->orientation.y,
                                imu_msg->orientation.z,
                                imu_msg->orientation.w));
}

void arduinisCallback(const grudsby_lowlevel::ArduinoResponse::ConstPtr& ard_msg)
{
  odom_tracker_.updateOdom(ard_msg->encoderLeft, ard_msg->encoderRight);
}

void gpsOdomCallback(const nav_msgs::Odometry::ConstPtr& gps_msg)
{
  Vector3 position = Vector3(gps_msg->pose.pose.position.x, gps_msg->pose.pose.position.y, gps_msg->pose.pose.position.z);
  Quaternion orientation = Quaternion(gps_msg->pose.pose.orientation.x, 
                                      gps_msg->pose.pose.orientation.y,
                                      gps_msg->pose.pose.orientation.z,
                                      gps_msg->pose.pose.orientation.w);
  odom_tracker_.updateGPS(position, orientation);
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "grudsby_odom_heading_node");

  ros::NodeHandle n;

  ros::Subscriber imuRawSub = n.subscribe("imu/data_raw", 1000, imuCallback);

  ros::Subscriber magSub = n.subscribe("imu/mag", 1000, imuMagCallback);

  ros::Subscriber gpuOdomSub = n.subscribe("odometry/gps", 1000, gpsOdomCallback);

  ros::Subscriber ardSub = n.subscribe("grudsby/arduino_response", 1000, arduinisCallback);


  mag_pub_ = n.advertise<sensor_msgs::MagneticField>("imu/mag_with_odom", 1000);
  int count = 0;
  ros::Rate loop_rate(1000);
  while (ros::ok()) {

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}