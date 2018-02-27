#!/usr/bin/env python

import rospy
import roslib
import tf
roslib.load_manifest('sim_repeater')

from laser_geometry import LaserProjection
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import MagneticField
from grudsby_lowlevel.msg import ArduinoResponse

#############################################################################
class DiffTf:
#############################################################################
    num_imu_packets = 0
    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("sim_repeater")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)

        #laser geometry projector#
        self.projector = LaserProjection()
        #### parameters #######
        self.rate = 100
        self.w = 0.508 #wheelbase len
        # subscriptions
        self.gpsFixPub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=10) 
        self.gpsPub = rospy.Publisher("/fix", NavSatFix, queue_size=10)
        self.imuPub = rospy.Publisher("/imu/data_raw", Imu, queue_size=10)
        self.magPub = rospy.Publisher("/imu/mag", MagneticField, queue_size=10)
        self.laserPub = rospy.Publisher("/tegra_stereo/points2", PointCloud2, queue_size=10)
        self.odomPub = rospy.Publisher("/grudsby/arduino_response", ArduinoResponse, queue_size=10)
        rospy.Subscriber("/odom_sim", Odometry, self.odomResponseCallback) 
        rospy.Subscriber("/laser/scan_sim", LaserScan, self.laserResponseCallback) 
        rospy.Subscriber("/fix_sim", NavSatFix, self.responseCallback)
        rospy.Subscriber("/imu/data_sim", Imu, self.imuResponseCallback)
        rospy.Subscriber("/mag", Vector3Stamped, self.magResponseCallback)
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
    #############################################################################
    def update(self):
    #############################################################################
        pass

    #############################################################################
    def laserResponseCallback(self, msg):
    #############################################################################
        scan_in = msg
        cloud_out = self.projector.projectLaser(scan_in)
        cloud_out.header.frame_id = "stereo_camera"
        self.laserPub.publish(cloud_out)

    #############################################################################
    def responseCallback(self, msg):
    #############################################################################
        gps_msg = NavSatFix()
        gps_msg.header = msg.header
        gps_msg.header.frame_id = "gps"
        gps_msg.status.status = 2
        gps_msg.latitude = msg.latitude
        gps_msg.longitude = msg.longitude
        gps_msg.altitude = msg.altitude
        gps_msg.position_covariance_type = msg.position_covariance_type
        if msg.status.status == 0: # no rtk
            gps_msg.position_covariance[0] = 80
            gps_msg.position_covariance[4] = 80
            gps_msg.position_covariance[8] = 150
        else:
            gps_msg.position_covariance[0] = 0.1
            gps_msg.position_covariance[4] = 0.1
            gps_msg.position_covariance[8] = 0.25
        self.gpsPub.publish(gps_msg)
        self.gpsFixPub.publish(gps_msg)

    #############################################################################
    def imuResponseCallback(self, msg):
    #############################################################################
        self.num_imu_packets += 1
        if self.num_imu_packets > 100:
            imu_msg = Imu()
            imu_msg.header = msg.header
	    imu_msg.header.frame_id = "imu_link"
	    imu_msg.orientation = msg.orientation
	    imu_msg.angular_velocity = msg.angular_velocity
	    imu_msg.linear_acceleration = msg.linear_acceleration
	    imu_msg.orientation_covariance[0] = 0.1
	    imu_msg.orientation_covariance[4] = 0.1
	    imu_msg.orientation_covariance[8] = 0.1
	    imu_msg.angular_velocity_covariance[0] = 0.5
	    imu_msg.angular_velocity_covariance[4] = 0.5
	    imu_msg.angular_velocity_covariance[8] = 0.5
	    imu_msg.linear_acceleration_covariance[0] = 0.1
	    imu_msg.linear_acceleration_covariance[4] = 0.1
	    imu_msg.linear_acceleration_covariance[8] = 0.1
	
	    self.imuPub.publish(imu_msg)


    #############################################################################
    def odomResponseCallback(self, msg):
    #############################################################################
         ard_response = ArduinoResponse()
         ard_response.linearX = msg.twist.twist.linear.x
         ard_response.angularZ = -msg.twist.twist.angular.z
         self.odomPub.publish(ard_response)

    #############################################################################
    def magResponseCallback(self, msg):
    #############################################################################

        mag_msg = MagneticField()
        mag_msg.header = msg.header
        mag_msg.header.frame_id = "imu_link"
        mag_msg.magnetic_field.x = msg.vector.x
        mag_msg.magnetic_field.y = msg.vector.y
        mag_msg.magnetic_field.z = 0.0 
        mag_msg.magnetic_field_covariance[0] = 0.1
        mag_msg.magnetic_field_covariance[4] = 0.1
        mag_msg.magnetic_field_covariance[8] = 0.1

        self.magPub.publish(mag_msg)




#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
