#!/usr/bin/env python

import rospy
import roslib
import tf

from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference


#############################################################################
class GpsSpoof:
#############################################################################
    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("gps_spoof")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.rate = 1
        
        self.fix_pub = rospy.Publisher('fix', NavSatFix, queue_size=1)
       
    #############################################################################
    def spin(self):
    #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            current_time = rospy.get_rostime()
            frame_id = 'gps'
            latitude = 40.444539 
            longitude = -79.940777
 
            current_fix = NavSatFix()
            current_fix.header.stamp = current_time
            current_fix.header.frame_id = frame_id
            current_time_ref = TimeReference()
            current_time_ref.header.stamp = current_time
            current_time_ref.header.frame_id = frame_id
            current_time_ref.source = frame_id
            current_fix.status.status = NavSatStatus.STATUS_FIX
            current_fix.status.service = NavSatStatus.SERVICE_GPS
            current_fix.latitude = latitude
            current_fix.longitude = longitude
            current_fix.position_covariance[0] = 0.1
            current_fix.position_covariance[4] = 0.1
            current_fix.position_covariance[8] = 0.25
            current_fix.position_covariance_type = \
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            current_fix.altitude = 0.0
            
            self.fix_pub.publish(current_fix)
 
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
        gps_spoof = GpsSpoof()
        gps_spoof.spin()
    except rospy.ROSInterruptException:
        pass
