#include "GrudsbyImu.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include <string>
#include <fstream>
#include <iostream>
#include <math.h>

bool run_calibration_;

std::string calib_location_;

int num_gyro_readings = 0;
int num_magn_readings = 0;
const int max_gyro_readings = 100;
const int max_magn_readings = 600;

double magn_x_bias_ = 0;
double magn_y_bias_ = 0;
double magn_z_bias_ = 0;
double gyro_x_bias_ = 0;
double gyro_y_bias_ = 0;
double gyro_z_bias_ = 0;

ros::Publisher velPub;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "grudsby_imu_node");

  ros::NodeHandle n;

  ros::Publisher imuRawPub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

  ros::Publisher magPub = n.advertise<sensor_msgs::MagneticField>("imu/mag", 1000);

  ros::Rate loop_rate(1000);

  if (!n.getParam ("ImuDriver/do_calibration", run_calibration_)) {
    run_calibration_ = false;
  }
  if (run_calibration_) {
    velPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  }

  if (!n.getParam ("ImuDriver/calibration_file_location", calib_location_)) {
    ROS_ERROR("Didn't specify a calibration file location");
    calib_location_ = "";
  }
  else {
    std::ifstream calibFile(calib_location_.c_str(), std::ios::binary);
    if (!calibFile) {
      ROS_ERROR("Could not open calibration file %s for IMU", calib_location_.c_str());
      magn_x_bias_ = 0;
      magn_y_bias_ = 0;
      magn_z_bias_ = 0;
      gyro_x_bias_ = 0;
      gyro_y_bias_ = 0;
      gyro_z_bias_ = 0;
    }
    else {
      char temp[256];
      calibFile.getline(temp,256);
      magn_x_bias_ = std::stod(temp);
      calibFile.getline(temp,256);
      magn_y_bias_ = std::stod(temp);
      calibFile.getline(temp,256);
      magn_z_bias_ = std::stod(temp);
      calibFile.getline(temp,256);
      gyro_x_bias_ = std::stod(temp);
      calibFile.getline(temp,256);
      gyro_y_bias_ = std::stod(temp);
      calibFile.getline(temp,256);
      gyro_z_bias_ = std::stod(temp);
      calibFile.close();
    }
  }  
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
    double accum_gyro_x = 0;
    double accum_gyro_y = 0;
    double accum_gyro_z = 0;
    double max_magn_x = -1000000;
    double min_magn_x = 1000000;
    double max_magn_y = -1000000; 
    double min_magn_y = 1000000;
    double accum_magn_z = 0;
    

    int count = 0;  
    bool keepRunning = true;
    while (ros::ok() && keepRunning)
    {
      if (run_calibration_) {
        //ROS_ERROR("RUNNING...");
        if (num_gyro_readings == 0) {
          geometry_msgs::Twist msg;
          msg.angular.z = 0.0;
          velPub.publish(msg);
        }
        if (num_gyro_readings < max_gyro_readings) {
          accum_gyro_x += Gyro.readX();
          accum_gyro_y += Gyro.readY();
          accum_gyro_z += Gyro.readZ();
          num_gyro_readings++;
        }
        else {
          if (num_magn_readings < max_magn_readings) {
              geometry_msgs::Twist msg;
              msg.angular.z = 1.89;
              velPub.publish(msg);

            if (num_magn_readings == 0) {
              ROS_ERROR("Finished Gyro Readings");
	    }
            double tempX = Mag.readX();
            double tempY = Mag.readY();
            if (tempX > max_magn_x)
              max_magn_x = tempX;
            if (tempX < min_magn_x)
              min_magn_x = tempX;
            if (tempY > max_magn_y)
              max_magn_y = tempY;
            if (tempY < min_magn_y)
              min_magn_y = tempY;
            accum_magn_z += Mag.readZ(); 
            num_magn_readings++;
          }
          else {
            geometry_msgs::Twist msg;
            msg.angular.z = 0.0;
            velPub.publish(msg);
            magn_x_bias_ = (max_magn_x + min_magn_x)/2.0;
            magn_y_bias_ = (max_magn_y + min_magn_y)/2.0;
            magn_z_bias_ = accum_magn_z / static_cast<double>(max_magn_readings); 
            gyro_x_bias_ = accum_gyro_x / static_cast<double>(max_gyro_readings);
            gyro_y_bias_ = accum_gyro_y / static_cast<double>(max_gyro_readings);
            gyro_z_bias_ = accum_gyro_z / static_cast<double>(max_gyro_readings);
            std::ofstream calibOutput(calib_location_.c_str(), std::ios::binary);
            if (calibOutput) {
              calibOutput << magn_x_bias_ << std::endl;
              calibOutput << magn_y_bias_ << std::endl;
              calibOutput << magn_z_bias_ << std::endl;
              calibOutput << gyro_x_bias_ << std::endl;
              calibOutput << gyro_y_bias_ << std::endl;
              calibOutput << gyro_z_bias_ << std::endl;
              calibOutput.close();
            }
            else {
              ROS_ERROR("Couldn't write to calibration file");
            }
            ROS_ERROR("Finished Calibration procedure");
            keepRunning = false;
          }
        }    
      }     
      else {
        sensor_msgs::Imu imu;
        imu.header.stamp = ros::Time::now();
        imu.header.frame_id = "imu_link";
        
        imu.angular_velocity.y = Gyro.readX() - gyro_x_bias_;
        imu.angular_velocity.x = -1 * (Gyro.readY() - gyro_y_bias_);
        imu.angular_velocity.z = Gyro.readZ() - gyro_z_bias_;
        imu.angular_velocity_covariance[0] = 0.000001; // NOTE: NEED TO ADJUST COVARIANCES
        imu.angular_velocity_covariance[4] = 0.000001; // NOTE: NEED TO ADJUST COVARIANCES
        imu.angular_velocity_covariance[8] = 0.000001; // NOTE: NEED TO ADJUST COVARIANCES
        
        
        imu.linear_acceleration.y = Accel.readX();
        imu.linear_acceleration.x = -1 * Accel.readY();
        imu.linear_acceleration.z = Accel.readZ();
        imu.linear_acceleration_covariance[0] = 0.0001; // NOTE: NEED TO ADJUST COVARIANCES
        imu.linear_acceleration_covariance[4] = 0.0001; // NOTE: NEED TO ADJUST COVARIANCES
        imu.linear_acceleration_covariance[8] = 0.0001; // NOTE: NEED TO ADJUST COVARIANCES
        imu.orientation_covariance[0] = 0.0001; // NOTE: NEED TO ADJUST COVARIANCES
        imu.orientation_covariance[4] = 0.0001; // NOTE: NEED TO ADJUST COVARIANCES
        imu.orientation_covariance[8] = 0.0001; // NOTE: NEED TO ADJUST COVARIANCES
        
        imuRawPub.publish(imu);
        
        sensor_msgs::MagneticField mag;
        mag.header.stamp = ros::Time::now();
        mag.header.frame_id = "imu_link"; 
        mag.magnetic_field.y = (-(Mag.readX() - magn_x_bias_)); // Convert to Gauss from mG
        mag.magnetic_field.x = -1* (Mag.readY() - magn_y_bias_);
        mag.magnetic_field.z = -1* (Mag.readZ() - magn_z_bias_);
        mag.magnetic_field_covariance[0] = 0.0001;
        mag.magnetic_field_covariance[4] = 0.0001;
        mag.magnetic_field_covariance[8] = 0.0001;
        magPub.publish(mag);
        
      }
      ros::spinOnce();

      loop_rate.sleep();
      ++count;
    }
  }
  return 0;
}
