#!/usr/bin/env python

import rospy
import roslib
import tf
import math
#roslib.load_manifest('scan_repeater')

from sensor_msgs.msg import LaserScan

MAX_RANGE = 10

class DiffTf:
    def __init__(self):
        rospy.init_node("scan_repeater")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        self.rate = 100
        self.laserPub = rospy.Publisher("scan_max", LaserScan, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.laserResponseCallback) 
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
        cloud_out = LaserScan()
        cloud_out = msg
        cloud_out.ranges = list ( cloud_out.ranges )
        for i in range ( len ( cloud_out.ranges ) ):
            if math.isnan ( cloud_out.ranges[ i ] ) :
                cloud_out.ranges [ i ] = MAX_RANGE
        cloud_out.ranges = tuple ( cloud_out.ranges )
        
        self.laserPub.publish(cloud_out)

if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
