#!/usr/bin/env python

import rospy
import roslib
import tf
import math
#roslib.load_manifest('scan_repeater')

from sensor_msgs.msg import LaserScan

MAX_RANGE = 3.5

class DiffTf:
    def __init__(self):
        rospy.init_node("scan_repeater")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        self.rate = 100
        self.laserPub = rospy.Publisher("scan_clipped", LaserScan, queue_size=10)
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

    def laserResponseCallback(self, msg):
    ## Clip to MAX_RANGE, and set  edges to MAX_RANGE to avoid stuck pixels######
        cloud_out = msg
        ranges = list ( msg.ranges )
        for i in range ( len ( ranges ) ) :
          if ranges [ i ] > MAX_RANGE:
            ranges [ i ] = MAX_RANGE
          
        for i in range (  10    ): ranges[i] = MAX_RANGE
        for i in range ( -10, 0 ): ranges[i] = MAX_RANGE
        cloud_out.ranges = tuple (ranges) 
        self.laserPub.publish(cloud_out)


if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
