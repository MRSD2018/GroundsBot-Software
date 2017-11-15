#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include <math.h>







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




int main(int argc, char **argv) {
  ros::init(argc, argv, "grudsby_imu_node");

  ros::NodeHandle n;

  ros::Publisher imuRawPub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);

  ros::Publisher magPub = n.advertise<sensor_msgs::MagneticField>("imu/mag", 1000);

  ros::Rate loop_rate(100);
  double angle = 0;
  double angleRate = 0.5;
  int count = 0;  
  while (ros::ok())
  {
    angle += angleRate * 0.01;
    sensor_msgs::Imu imu;
    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = "imu_link";
    
    imu.angular_velocity.x = rand_normal(0, .005);
    imu.angular_velocity.y = rand_normal(0, .005);
    imu.angular_velocity.z = angleRate + rand_normal(0, .005);
    imu.angular_velocity_covariance[0] = 0.01; // NOTE: NEED TO ADJUST COVARIANCES
    imu.angular_velocity_covariance[4] = 0.01; // NOTE: NEED TO ADJUST COVARIANCES
    imu.angular_velocity_covariance[8] = 0.01; // NOTE: NEED TO ADJUST COVARIANCES
    imu.linear_acceleration.x = rand_normal(0, .015);
    imu.linear_acceleration.y = rand_normal(0, .015);
    imu.linear_acceleration.z = 9.81 + rand_normal(0, .015);
    imu.linear_acceleration_covariance[0] = 0.2; // NOTE: NEED TO ADJUST COVARIANCES
    imu.linear_acceleration_covariance[4] = 0.2; // NOTE: NEED TO ADJUST COVARIANCES
    imu.linear_acceleration_covariance[8] = 0.2; // NOTE: NEED TO ADJUST COVARIANCES
    imuRawPub.publish(imu);

    sensor_msgs::MagneticField mag;
    mag.header.stamp = ros::Time::now();
    mag.header.frame_id = "imu_link"; 
    mag.magnetic_field.x = 0.000032 * cos(-angle ) * rand_normal(1, .005);
    mag.magnetic_field.y = 0.000032 * sin(-angle ) * rand_normal(1, .005);
    mag.magnetic_field.z = rand_normal(0, .005);
    mag.magnetic_field_covariance[0] = 0.000001;
    mag.magnetic_field_covariance[4] = 0.000001;
    mag.magnetic_field_covariance[8] = 0.000001;
    magPub.publish(mag);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}