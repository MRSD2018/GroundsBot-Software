/* Copyright 2018 Grudsbois */

#include "OdomHeading.h"
#include <ros/console.h>
#include <ros/ros.h>
#define _USE_MATH_DEFINES

OdomHeading::OdomHeading() {}

OdomHeading::~OdomHeading() {
  odom_pos_ = Vector3(0.0, 0.0, 0.0);
  imu_ori_ = Quaternion(0.0, 0.0, 0.0, 1.0);
  odom_ori_yaw_ = 0.0;

  first_odom_received_ = false;
  heading_estimate_established_ = false;
  first_gps_received_ = false;
  th_heading_estimate_ = 0;
}

void OdomHeading::updateOdom(unsigned int left_wheel,
                             unsigned int right_wheel) {
  if (first_odom_received_) {
    double leftDist =
        WHEEL_RAD * (left_wheel - left_wheel_odom_) * 2.0 * M_PI / 4096.0;
    double rightDist =
        WHEEL_RAD * (right_wheel - right_wheel_odom_) * 2.0 * M_PI / 4096.0;
    Quaternion zHalfRot = Quaternion::FromEuler(
        0, 0, atan2(rightDist - leftDist, WHEELBASE_LEN) / 2.0);
    Quaternion zYawRot = Quaternion::FromEuler(0, 0, odom_ori_yaw_);
    Quaternion tempOri = imu_ori_ * zYawRot;
    tempOri = zHalfRot * tempOri;
    double avgDist = (rightDist + leftDist) / 2.0;
    odom_pos_ += tempOri * Vector3(avgDist, 0, 0);
    tempOri = zHalfRot * tempOri;
    Vector3 euler = Quaternion::ToEuler(tempOri);
    odom_ori_yaw_ = euler.Z;
  } else {
    first_odom_received_ = true;
    ROS_ERROR("ODOM RECEIVED");
  }
  left_wheel_odom_ = left_wheel;
  right_wheel_odom_ = right_wheel;
}

void OdomHeading::updateGPS(Vector3 pos, Quaternion ori) {
  if (first_gps_received_) {
    Vector3 deltaGPS = pos = last_gps_pos_;
    double dist = Vector3::Magnitude(deltaGPS);
    if (dist > GPS_DIST_THRESHOLD) {
      double odomDist = Vector3::Magnitude(odom_pos_);
      if (abs(odomDist - dist) < GPS_ODOM_DIFF_THRESHOLD) {
        th_heading_estimate_ =
            atan2(odom_pos_.Y, odom_pos_.X) - atan2(deltaGPS.Y, deltaGPS.X);
        odom_pos_ = Vector3(0.0, 0.0, 0.0);
        odom_ori_yaw_ = 0;
        last_gps_pos_ = pos;
        last_gps_ori_ = ori;
        if (!heading_estimate_established_) {
          heading_estimate_established_ = true;
          ROS_ERROR("Heading Estimate Received");
        }
      }
    }
  } else {
    last_gps_pos_ = pos;
    last_gps_ori_ = ori;
    first_gps_received_ = true;
    ROS_ERROR("GPS RECEIVED");
  }
}

void OdomHeading::updateIMU(Quaternion oriIMU) {
  Vector3 euler = Quaternion::ToEuler(oriIMU);
  euler.Z = 0;
  imu_ori_ = Quaternion::FromEuler(euler);
}

Vector3 OdomHeading::processMagMessage(Vector3 magVector) {
  if (heading_estimate_established_) {
    Vector3 ret = Vector3(
        0.19 * cos(th_heading_estimate_ + odom_ori_yaw_ + M_PI / 2.0),
        0.19 * sin(th_heading_estimate_ + odom_ori_yaw_ + M_PI / 2.0), 0.0);
    return ret;
  } else {
    return magVector;
  }
}