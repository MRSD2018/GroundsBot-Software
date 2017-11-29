#ifndef ODOM_HEADING_H
#define ODOM_HEADING_H

#include <math.h>
#include "Vector3.hpp"
#include "Quaternion.hpp"
#include "Matrix3x3.hpp"

class OdomHeading {
public:
  OdomHeading();
  ~OdomHeading();
  void updateOdom(unsigned int left_wheel, unsigned int right_wheel);
  void updateGPS(Vector3 pos, Quaternion ori);
  void updateIMU(Quaternion ori);
  Vector3 processMagMessage(Vector3 magVector);

private:
  bool first_odom_received_, heading_estimate_established_, first_gps_received_;
  unsigned int left_wheel_odom_, right_wheel_odom_;
  Vector3 odom_pos_;
  Quaternion imu_ori_;
  double th_heading_estimate_;
  double odom_ori_yaw_;
  Vector3 last_gps_pos_;
  Quaternion last_gps_ori_;
  const double WHEELBASE_LEN = 0.508;
  const double WHEEL_RAD = 0.127;
  const double GPS_DIST_THRESHOLD = 2.0;
  const double GPS_ODOM_DIFF_THRESHOLD = 0.5;
};

#endif