#!/usr/bin/env python

import rospy
import roslib
import math
roslib.load_manifest('imu_repeater')


from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

#############################################################################
class DiffTf:
#############################################################################
    
    totalVel = 0
    rtkEnabled = 0 # 0 = no RTK, don't trust GPS for yaw, 1 = yes RTK
    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("imu_repeater")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        #### parameters #######
        self.rate = 200

        # subscriptions
     
        self.imuPub = rospy.Publisher("/imu/data_filtered_covariance", Imu, queue_size=10)
        rospy.Subscriber("/imu/data", Imu, self.imuResponseCallback)
        rospy.Subscriber("/odometry/filtered_map", Odometry, self.gpsResponseCallback)
        rospy.Subscriber("/fix", NavSatFix, self.fixResponseCallback)
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
    def imuResponseCallback(self, msg):
    #############################################################################
        
        
        imu_msg = Imu()
        imu_msg.header = msg.header
        imu_msg.orientation = msg.orientation
        imu_msg.angular_velocity = msg.angular_velocity
        imu_msg.linear_acceleration = msg.linear_acceleration
        imu_msg.orientation_covariance[0] = msg.orientation_covariance[0]
        imu_msg.orientation_covariance[4] = msg.orientation_covariance[4]
        if self.rtkEnabled == 0:
            imu_msg.orientation_covariance[8] = msg.orientation_covariance[8]
        else:
            imu_msg.orientation_covariance[8] = msg.orientation_covariance[8] + 99999/(1+math.exp(-8*(abs(self.totalVel) - 2))) # sigmoid function to mix yaw from the IMU
        
        imu_msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[0]
        imu_msg.angular_velocity_covariance[4] = msg.angular_velocity_covariance[4]
        imu_msg.angular_velocity_covariance[8] = msg.angular_velocity_covariance[8]
        imu_msg.linear_acceleration_covariance[0] = msg.linear_acceleration_covariance[0]
        imu_msg.linear_acceleration_covariance[4] = msg.linear_acceleration_covariance[4]
        imu_msg.linear_acceleration_covariance[8] = msg.linear_acceleration_covariance[8]
        rospy.logerr("Vel: %f   Z Rot Covariance: %f", self.totalVel, imu_msg.orientation_covariance[8])
        self.imuPub.publish(imu_msg)

    #############################################################################
    def gpsResponseCallback(self, msg):
    #############################################################################
        self.totalVel = msg.twist.twist.linear.x
        #rospy.logerr("Velocity: %f", self.totalVel)


    #############################################################################
    def fixResponseCallback(self, msg):
    #############################################################################
        if msg.status.status == 0: # no rtk
            self.rtkEnabled = 0
        else:
            self.rtkEnabled = 1 

#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
