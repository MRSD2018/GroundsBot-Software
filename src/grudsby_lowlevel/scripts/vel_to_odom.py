#!/usr/bin/env python

import rospy
import roslib
roslib.load_manifest('differential_drive')
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int32
from grudsby_lowlevel.msg import ArduinoResponse

#############################################################################
class DiffTf:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("vel_to_odom")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.rate = 100
        
  
        
        
        # subscriptions
        rospy.Subscriber("/grudsby/arduino_response", ArduinoResponse, self.responseCallback)
        rospy.Subscriber("rwheel", Int32, self.rwheelCallback)

        self.odomPub = rospy.Publisher("/grudsby/odometry", Odometry, queue_size=10)
        
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
        odom_msg = Odometry()
        odom_msg.twist.twist.linear.x = msg.linearX
        odom_msg.twist.twist.angular.z = msg.angularZ
        odom_msg.twist.covariance[0] = 0.0004
	odom_msg.twist.covariance[7] = 0.0004
        odom_msg.twist.covariance[14] = 0.0004
	odom_msg.twist.covariance[21] = 0.0004
        odom_msg.twist.covariance[28] = 0.0004
        odom_msg.twist.covariance[35] = 0.0004

	self.odomPub.publish(odom_msg)
        
    #############################################################################
    def rwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.rmult = self.rmult - 1
            
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc

#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
