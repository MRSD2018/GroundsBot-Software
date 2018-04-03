#!/usr/bin/env python

import rospy
import roslib
import tf
import math
#roslib.load_manifest('scan_repeater')

from sensor_msgs.msg import LaserScan

MAX_RANGE =  8

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
        window_size = 20
        
        cloud_out = LaserScan()
        cloud_out = msg
        #cloud_out.ranges = list ( cloud_out.ranges )
        #for i in range ( len ( cloud_out.ranges ) ):
        fix_nans = range ( len ( cloud_out.ranges ) )
        for i in range ( len ( cloud_out.ranges ) ):
            if math.isnan ( msg.ranges[ i ] ) :
                fix_nans [ i ] = MAX_RANGE
            else:
                fix_nans [ i ] = msg.ranges [ i ]

        output = [ MAX_RANGE for i in range ( len ( fix_nans ) ) ]
        #for i in range ( 200 , len ( fix_nans )-200 ):
        for i in range ( window_size , len ( fix_nans )-window_size ):
            neighborhood = fix_nans [ i-window_size : i + window_size + 1] 
            neighborhood.sort ( )
            output [ i ] = neighborhood [ int (  math.floor ( window_size / 2 )  ) ]

        cloud_out.ranges = tuple (output) 
        self.laserPub.publish(cloud_out)

if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
