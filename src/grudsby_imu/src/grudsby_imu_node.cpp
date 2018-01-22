/* Copyright 2018 Grudsbois */
#include <math.h>

#include <fstream>
#include <iostream>
#include <string>

#include "GrudsbyImu.h"
#include "geometry_msgs/Twist.h"

#include "grudsby_lowlevel/ArduinoResponse.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/String.h"

bool run_calibration_;

std::string calib_location_;

int num_gyro_readings = 0;
int num_magn_readings = 0;
const int kMaxGyroReadings = 100;
const int kMaxmagnReadings = 600;

double magn_x_bias_ = 0;
double magn_y_bias_ = 0;
double magn_z_bias_ = 0;
double gyro_x_bias_ = 0;
double gyro_y_bias_ = 0;
double gyro_z_bias_ = 0;

int first_enc_left_;
int last_enc_left_;
int first_enc_right_;
int last_enc_right_;
bool captured_first_enc = false;

ros::Publisher vel_pub;

void arduinisCallback(const grudsby_lowlevel::ArduinoResponse::ConstPtr& ard_msg)
{
  if (!captured_first_enc)
  {
    captured_first_enc = true;
    first_enc_left_ = ard_msg->encoderLeft;
    first_enc_right_ = ard_msg->encoderRight;
  }
  else
  {
    last_enc_left_ = ard_msg->encoderLeft;
    last_enc_right_ = ard_msg->encoderRight;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grudsby_imu_node");

  ros::NodeHandle n;

  ros::Publisher imu_raw_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

  ros::Publisher mag_pub = n.advertise<sensor_msgs::MagneticField>("imu/mag", 1000);

  ros::Subscriber arduino_sub = n.subscribe("grudsby/arduino_response", 1000, arduinisCallback);

  ros::Rate loop_rate(1000);

  if (!n.getParam("ImuDriver/do_calibration", run_calibration_))
  {
    run_calibration_ = false;
  }
  if (run_calibration_)
  {
    vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    /*  USEFUL CODE FOR CALIBRATING SPEEDS! LEAVE IN CODE...
    ros::Duration(5.0).sleep();
    geometry_msgs::Twist msg;
    msg.linear.x = -1.0;
    msg.angular.z = 0.0;
    velPub.publish(msg);
    ros::spinOnce();
    for (int i = 0; i < 20; i++) {
      ros::Duration(1.0).sleep();
      velPub.publish(msg);
      ros::spinOnce();
    }
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    velPub.publish(msg);
    ros::spinOnce();
    ROS_ERROR("Left Wheel Vel (ticks/sec)=
    %f",static_cast<double>(last_enc_left_-first_enc_left_)/20.0);
    ROS_ERROR("Right Wheel Vel (ticks/sec)=
    %f",static_cast<double>(last_enc_right_-first_enc_right_)/20.0);
    */
  }

  if (!n.getParam("ImuDriver/calibration_file_location", calib_location_))
  {
    ROS_ERROR("Didn't specify a calibration file location");
    calib_location_ = "";
  }
  else
  {
    std::ifstream calib_file(calib_location_.c_str(), std::ios::binary);
    if (!calib_file)
    {
      ROS_ERROR("Could not open calibration file %s for IMU", calib_location_.c_str());
      magn_x_bias_ = 0;
      magn_y_bias_ = 0;
      magn_z_bias_ = 0;
      gyro_x_bias_ = 0;
      gyro_y_bias_ = 0;
      gyro_z_bias_ = 0;
    }
    else
    {
      char temp[256];
      calib_file.getline(temp, 256);
      magn_x_bias_ = std::stod(temp);
      calib_file.getline(temp, 256);
      magn_y_bias_ = std::stod(temp);
      calib_file.getline(temp, 256);
      magn_z_bias_ = std::stod(temp);
      calib_file.getline(temp, 256);
      gyro_x_bias_ = std::stod(temp);
      calib_file.getline(temp, 256);
      gyro_y_bias_ = std::stod(temp);
      calib_file.getline(temp, 256);
      gyro_z_bias_ = std::stod(temp);
      calib_file.close();
    }
  }
  bool init_successful = true;
  GrudsbyImuAccel accel = GrudsbyImuAccel();
  if (!accel.begin())
  {
    ROS_ERROR("Accelerometer did not initialize succesfully...");
    init_successful = false;
  }
  GrudsbyImuMag mag = GrudsbyImuMag();
  if (!mag.begin())
  {
    ROS_ERROR("magnetometer did not initialize succesfully...");
    init_successful = false;
  }
  GrudsbyImuGyro gyro = GrudsbyImuGyro();
  if (!gyro.begin())
  {
    ROS_ERROR("Gyroscope did not initialize succesfully...");
    init_successful = false;
  }
  if (init_successful)
  {
    double accum_gyro_x = 0;
    double accum_gyro_y = 0;
    double accum_gyro_z = 0;
    double max_magn_x = -1000000;
    double min_magn_x = 1000000;
    double max_magn_y = -1000000;
    double min_magn_y = 1000000;
    double accum_magn_z = 0;

    int count = 0;
    bool keep_running = true;
    while (ros::ok() && keep_running)
    {
      if (run_calibration_)
      {
        // ROS_ERROR("RUNNING...");
        if (num_gyro_readings == 0)
        {
          geometry_msgs::Twist msg;
          msg.angular.z = 0.0;
          vel_pub.publish(msg);
        }
        if (num_gyro_readings < kMaxGyroReadings)
        {
          accum_gyro_x += gyro.readX();
          accum_gyro_y += gyro.readY();
          accum_gyro_z += gyro.readZ();
          num_gyro_readings++;
        }
        else
        {
          if (num_magn_readings < kMaxmagnReadings)
          {
            geometry_msgs::Twist msg;
            msg.angular.z = 1.89;
            vel_pub.publish(msg);

            if (num_magn_readings == 0)
            {
              ROS_ERROR("Finished Gyro Readings");
            }
            double temp_x = mag.readX();
            double temp_y = mag.readY();
            if (temp_x > max_magn_x) {
              max_magn_x = temp_x;
}
            if (temp_x < min_magn_x) {
              min_magn_x = temp_x;
}
            if (temp_y > max_magn_y) {
              max_magn_y = temp_y;
}
            if (temp_y < min_magn_y) {
              min_magn_y = temp_y;
}
            accum_magn_z += mag.readZ();
            num_magn_readings++;
          }
          else
          {
            geometry_msgs::Twist msg;
            msg.angular.z = 0.0;
            vel_pub.publish(msg);
            magn_x_bias_ = (max_magn_x + min_magn_x) / 2.0;
            magn_y_bias_ = (max_magn_y + min_magn_y) / 2.0;
            magn_z_bias_ = accum_magn_z / static_cast<double>(kMaxmagnReadings);
            gyro_x_bias_ = accum_gyro_x / static_cast<double>(kMaxGyroReadings);
            gyro_y_bias_ = accum_gyro_y / static_cast<double>(kMaxGyroReadings);
            gyro_z_bias_ = accum_gyro_z / static_cast<double>(kMaxGyroReadings);
            std::ofstream calib_output(calib_location_.c_str(), std::ios::binary);
            if (calib_output)
            {
              calib_output << magn_x_bias_ << std::endl;
              calib_output << magn_y_bias_ << std::endl;
              calib_output << magn_z_bias_ << std::endl;
              calib_output << gyro_x_bias_ << std::endl;
              calib_output << gyro_y_bias_ << std::endl;
              calib_output << gyro_z_bias_ << std::endl;
              calib_output.close();
            }
            else
            {
              ROS_ERROR("Couldn't write to calibration file");
            }
            ROS_ERROR("Finished Calibration procedure");
            keep_running = false;
          }
        }
      }
      else
      {
        sensor_msgs::Imu imu;
        imu.header.stamp = ros::Time::now();
        imu.header.frame_id = "imu_link";

        imu.angular_velocity.y = gyro.readX() - gyro_x_bias_;
        imu.angular_velocity.x = -1 * (gyro.readY() - gyro_y_bias_);
        imu.angular_velocity.z = gyro.readZ() - gyro_z_bias_;
        imu.angular_velocity_covariance[0] = 0.01;  // NOTE: NEED TO ADJUST COVARIANCES
        imu.angular_velocity_covariance[4] = 0.01;  // NOTE: NEED TO ADJUST COVARIANCES
        imu.angular_velocity_covariance[8] = 0.01;  // NOTE: NEED TO ADJUST COVARIANCES

        imu.linear_acceleration.y = accel.readX();
        imu.linear_acceleration.x = -1 * accel.readY();
        imu.linear_acceleration.z = accel.readZ();
        imu.linear_acceleration_covariance[0] = 0.1;  // NOTE: NEED TO ADJUST COVARIANCES
        imu.linear_acceleration_covariance[4] = 0.1;  // NOTE: NEED TO ADJUST COVARIANCES
        imu.linear_acceleration_covariance[8] = 0.1;  // NOTE: NEED TO ADJUST COVARIANCES
        imu.orientation_covariance[0] = 0.1;          // NOTE: NEED TO ADJUST COVARIANCES
        imu.orientation_covariance[4] = 0.1;          // NOTE: NEED TO ADJUST COVARIANCES
        imu.orientation_covariance[8] = 0.1;          // NOTE: NEED TO ADJUST COVARIANCES

        imu_raw_pub.publish(imu);

        sensor_msgs::MagneticField mag_msg;
        mag_msg.header.stamp = ros::Time::now();
        mag_msg.header.frame_id = "imu_link";
        mag_msg.magnetic_field.y = (-(mag.readX() - magn_x_bias_));  // Convert to Gauss from mG
        mag_msg.magnetic_field.x = -1 * (mag.readY() - magn_y_bias_);
        mag_msg.magnetic_field.z = -1 * (mag.readZ() - magn_z_bias_);
        mag_msg.magnetic_field_covariance[0] = 0.1;
        mag_msg.magnetic_field_covariance[4] = 0.1;
        mag_msg.magnetic_field_covariance[8] = 0.1;
        mag_pub.publish(mag_msg);
      }
      ros::spinOnce();

      loop_rate.sleep();
      ++count;
    }
  }
  return 0;
}
