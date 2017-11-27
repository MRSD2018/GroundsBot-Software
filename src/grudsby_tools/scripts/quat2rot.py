#!/usr/bin/env python

import rospy
import roslib
import tf
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from grudsby_lowlevel.msg import ArduinoResponse
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

#############################################################################
class Quat2Rot:
#############################################################################
    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("quat2rot")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.rate = 100
        
       
        
        
        # subscriptions
        rospy.Subscriber("/imu/data", Imu, self.ImuCallback)
        rospy.Subscriber("/odometry/filtered", Odometry, self.OdometryCallback)   


        self.rotPub = rospy.Publisher('grudsby/debug/IMUrot', Vector3, queue_size=10)
        self.filtPub = rospy.Publisher('grudsby/debug/IMUrotfilt', Vector3, queue_size=10)
       
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
    def ImuCallback(self, msg):
    #############################################################################
        quat = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)
    
        rots = Vector3() 
        rots.x = euler[0]
        rots.y = euler[1]
        rots.z = euler[2]

        self.rotPub.publish(rots)
    

        
    #############################################################################
    def OdometryCallback(self, msg):
    #############################################################################
        quat = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)

        rots = Vector3() 
        rots.x = euler[0]
        rots.y = euler[1]
        rots.z = euler[2]

        self.filtPub.publish(rots)
        
#############################################################################
#############################################################################


if __name__ == '__main__':
    """ main """
    try:
        quat2rot = Quat2Rot()
        quat2rot.spin()
    except rospy.ROSInterruptException:
        pass
