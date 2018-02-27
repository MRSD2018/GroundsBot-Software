#!/usr/bin/env python

import rospy
import roslib
import math
roslib.load_manifest('imu_repeater')


from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from grudsby_lowlevel.msg import ArduinoResponse
from std_msgs.msg import Bool

#############################################################################
class DiffTf:
#############################################################################
    lastLeftDiff = 0
    lastRightDiff = 0
    totalVel = 0
    leftWheelVel = 0
    rightWheelVel = 0
    rtkEnabled = 0 # 0 = no RTK, don't trust GPS for yaw, 1 = yes RTK
    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("imu_repeater")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        #### parameters #######
        self.rate = 200
        self.w = 0.508 #wheelbase len
        # subscriptions
        self.slipPub = rospy.Publisher("/grudsby/wheel_slip", Bool, queue_size = 0)
        self.imuPub = rospy.Publisher("/imu/data_filtered_covariance", Imu, queue_size=10)
        rospy.Subscriber("/imu/data", Imu, self.imuResponseCallback)
        rospy.Subscriber("/odometry/filtered_map", Odometry, self.gpsResponseCallback)
        rospy.Subscriber("/fix", NavSatFix, self.fixResponseCallback)
        rospy.Subscriber("/grudsby/arduino_response", ArduinoResponse, self.arduinoResponseCallback)


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
        #rospy.logerr("Vel: %f   Z Rot Covariance: %f", self.totalVel, imu_msg.orientation_covariance[8])
        self.imuPub.publish(imu_msg)

    #############################################################################
    def gpsResponseCallback(self, msg):
    #############################################################################
        self.totalVel = msg.twist.twist.linear.x
        #rospy.logerr("Velocity: %f", self.totalVel)
        self.rightWheelVel = msg.twist.twist.linear.x + msg.twist.twist.angular.z * self.w / 2.0
        self.leftWheelVel = msg.twist.twist.linear.x - msg.twist.twist.angular.z * self.w / 2.0


    #############################################################################
    def fixResponseCallback(self, msg):
    #############################################################################
        if msg.status.status == 0: # no rtk
            self.rtkEnabled = 0
        else:
            self.rtkEnabled = 1 

    #############################################################################
    def arduinoResponseCallback(self, msg):
    #############################################################################
        msg.linearX
        -msg.angularZ
        actualRightWheelVel = msg.linearX - msg.angularZ * self.w / 2.0
        actualLeftWheelVel = msg.linearX + msg.angularZ * self.w / 2.0
        compFilter = 0.25
        self.lastLeftDiff = compFilter * (actualLeftWheelVel - self.leftWheelVel) + (1 - compFilter) * self.lastLeftDiff
        self.lastRightDiff = compFilter * (actualRightWheelVel - self.rightWheelVel) + (1 - compFilter) * self.lastRightDiff
        slip = Bool()
        slip.data = False
        if (abs(self.lastLeftDiff) > 0.5 or abs(self.lastRightDiff) > 0.5):
            slip.data = True    
            #rospy.logerr("Wheel Slip Detected!")
        #rospy.logerr("left: %f    right: %f", self.lastLeftDiff, self.lastRightDiff)
        self.slipPub.publish(slip)

#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
