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
from std_msgs.msg import Float32
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
        
        self.w = 0.508 #wheelbase len

        
        
        # subscriptions
        rospy.Subscriber("/grudsby/arduino_response", ArduinoResponse, self.responseCallback)
        
        rospy.Subscriber("/cmd_vel", Twist, self.cmdCallback)

        self.odomPub = rospy.Publisher("/grudsby/odometry", Odometry, queue_size=10)
        self.lMotorPub = rospy.Publisher('/arduino/lwheel_vtarget', Float32, queue_size=10)
        self.rMotorPub = rospy.Publisher('/arduino/rwheel_vtarget', Float32, queue_size=10)
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
	odom_msg.header.frame_id = "base_link"
        odom_msg.child_frame_id = "base_link"
	self.odomPub.publish(odom_msg)
        
    #############################################################################
    def cmdCallback(self, msg):
    #############################################################################
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y
        self.right = 1.0 * self.dx + self.dr * self.w / 2 
        self.left = 1.0 * self.dx - self.dr * self.w / 2
        self.lMotorPub.publish(self.left)
        self.rMotorPub.publish(self.right)
        
#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
