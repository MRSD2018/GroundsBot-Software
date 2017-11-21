#include "GrudsbyImu.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include <math.h>


int main (int argc, char **argv)
{
  ros::init(argc, argv, "grudsby_imu_node");

  ros::NodeHandle n;

  ros::Publisher imuRawPub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

  ros::Publisher magPub = n.advertise<sensor_msgs::MagneticField>("imu/mag", 1000);

  ros::Rate loop_rate(100);
  
  bool initSuccessful = true; 
  GrudsbyImuAccel Accel = GrudsbyImuAccel();
  if (!Accel.begin()) {
    ROS_ERROR("Accelerometer did not initialize succesfully...");
    initSuccessful = false;
  }
  GrudsbyImuMag Mag = GrudsbyImuMag();
  if (!Mag.begin()) {
    ROS_ERROR("Magnetometer did not initialize succesfully...");
    initSuccessful = false;
  }
  GrudsbyImuGyro Gyro = GrudsbyImuGyro();
  if (!Gyro.begin()) {
    ROS_ERROR("Gyroscope did not initialize succesfully...");
    initSuccessful = false;
  }
  if (initSuccessful) {
    int count = 0;  
    while (ros::ok())
    {
      sensor_msgs::Imu imu;
      imu.header.stamp = ros::Time::now();
      imu.header.frame_id = "imu_link";
    
      imu.angular_velocity.x = Gyro.readX();
      imu.angular_velocity.y = Gyro.readY();
      imu.angular_velocity.z = Gyro.readZ();
      imu.angular_velocity_covariance[0] = 0.01; // NOTE: NEED TO ADJUST COVARIANCES
      imu.angular_velocity_covariance[4] = 0.01; // NOTE: NEED TO ADJUST COVARIANCES
      imu.angular_velocity_covariance[8] = 0.01; // NOTE: NEED TO ADJUST COVARIANCES
      imu.linear_acceleration.x = Accel.readX();
      imu.linear_acceleration.y = Accel.readY();
      imu.linear_acceleration.z = Accel.readZ();
      imu.linear_acceleration_covariance[0] = 0.2; // NOTE: NEED TO ADJUST COVARIANCES
      imu.linear_acceleration_covariance[4] = 0.2; // NOTE: NEED TO ADJUST COVARIANCES
      imu.linear_acceleration_covariance[8] = 0.2; // NOTE: NEED TO ADJUST COVARIANCES
      imuRawPub.publish(imu);

      sensor_msgs::MagneticField mag;
      mag.header.stamp = ros::Time::now();
      mag.header.frame_id = "imu_link"; 
      mag.magnetic_field.x = Mag.readX(); // Convert to Gauss from mG
      mag.magnetic_field.y = Mag.readY();
      mag.magnetic_field.z = Mag.readZ();
      mag.magnetic_field_covariance[0] = 0.000001;
      mag.magnetic_field_covariance[4] = 0.000001;
      mag.magnetic_field_covariance[8] = 0.000001;
      magPub.publish(mag);

      ros::spinOnce();

      loop_rate.sleep();
      ++count;
    }
  }
  return 0;
}
