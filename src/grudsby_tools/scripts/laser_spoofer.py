#!/usr/bin/env python

import rospy
import roslib
import tf

from sensor_msgs.msg import LaserScan


#############################################################################
class LaserSpoof:
#############################################################################
    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("laser_spoof")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.rate = 10
        
        self.laser_pub = rospy.Publisher('scan', LaserScan, queue_size=1)
       
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            current_time = rospy.get_rostime()
            frame_id = 'stereo_camera'

            scan = LaserScan()
            scan.header.stamp = current_time
            scan.header.frame_id = frame_id
            scan.scan_time = 0.1
            scan.time_increment = 0.1 
           
            self.laser_pub.publish(scan)
 
            self.update()
            r.sleep()
       
     
    #############################################################################
    def update(self):
    #############################################################################
        pass
            
            
        
#############################################################################
#############################################################################


if __name__ == '__main__':
    """ main """
    try:
        laser_spoof = LaserSpoof()
        laser_spoof.spin()
    except rospy.ROSInterruptException:
        pass
