#!/usr/bin/env python

import rospy
import roslib
roslib.load_manifest('temp_repeater')

from sensor_msgs.msg import NavSatFix

#############################################################################
class DiffTf:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("temp_repeater")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        #### parameters #######
        self.rate = 100
        self.w = 0.508 #wheelbase len
        # subscriptions
        rospy.Subscriber("/fix", NavSatFix, self.responseCallback)
        self.gpsPub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=10)
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
        gps_msg = NavSatFix()
        gps_msg.header = msg.header
        gps_msg.status = msg.status
        gps_msg.latitude = msg.latitude
        gps_msg.longitude = msg.longitude
        gps_msg.altitude = msg.altitude
        gps_msg.position_covariance_type = msg.position_covariance_type
        gps_msg.position_covariance[0] = 0.1
        gps_msg.position_covariance[4] = 0.1
        gps_msg.position_covariance[8] = 0.25
        self.gpsPub.publish(gps_msg)
        
        
#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
