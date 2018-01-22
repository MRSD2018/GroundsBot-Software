#!/usr/bin/env python

import rospy
import roslib
roslib.load_manifest('temp_repeater')

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

#############################################################################
class DiffTf:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("temp_repeater")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        #### parameters #######
        self.rate = 100
        self.w = 0.508 #wheelbase len
        # subscriptions
        
        self.gpsPub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=10)
        self.imuPub = rospy.Publisher("/imu/data_raw_new_covariance", Imu, queue_size=10)
        rospy.Subscriber("/fix", NavSatFix, self.responseCallback)
        rospy.Subscriber("/imu/data_raw", Imu, self.imuResponseCallback)
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
    def responseCallback(self, msg):
    #############################################################################
        gps_msg = NavSatFix()
        gps_msg.header = msg.header
        gps_msg.status = msg.status
        gps_msg.latitude = msg.latitude
        gps_msg.longitude = msg.longitude
        gps_msg.altitude = msg.altitude
        gps_msg.position_covariance_type = msg.position_covariance_type
        if msg.status.status == 0: # no rtk
            gps_msg.position_covariance[0] = 15
            gps_msg.position_covariance[4] = 15
            gps_msg.position_covariance[8] = 35
        else:
            gps_msg.position_covariance[0] = 0.1
            gps_msg.position_covariance[4] = 0.1
            gps_msg.position_covariance[8] = 0.25
        self.gpsPub.publish(gps_msg)
        

    #############################################################################
    def imuResponseCallback(self, msg):
    #############################################################################
        
        imu_msg = Imu()
        imu_msg.header = msg.header
        imu_msg.orientation = msg.orientation
        imu_msg.angular_velocity = msg.angular_velocity
        imu_msg.linear_acceleration = msg.linear_acceleration
        imu_msg.orientation_covariance[0] = 1
        imu_msg.orientation_covariance[4] = 1
        imu_msg.orientation_covariance[8] = 1
        imu_msg.angular_velocity_covariance[0] = 0.5
        imu_msg.angular_velocity_covariance[4] = 0.5
        imu_msg.angular_velocity_covariance[8] = 0.5
        imu_msg.linear_acceleration_covariance[0] = 0.1
        imu_msg.linear_acceleration_covariance[4] = 0.1
        imu_msg.linear_acceleration_covariance[8] = 0.1
        
        self.imuPub.publish(imu_msg)
        
#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
