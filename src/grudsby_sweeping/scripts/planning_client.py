#!/usr/bin/env python

import rospy
import roslib
import subprocess
import urllib2
roslib.load_manifest('planning_client')
from math import sin, cos, pi
from std_msgs.msg import Int32

#############################################################################
class DiffTf:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("planning_client")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.rate = 100
        
        args = ("{\"regionID\":\"sve\",\"coordinates\":[{\"lat\":40.44466972302791,\"lng\":-79
.94074523448944},{\"lat\":40.44450950754617,\"lng\":-79.94096517562866},{\"lat\":40.444376716046676,\"lng\":-79.94080424308777}]}","1")
	popen = subprocess.Popen(args, stdout=subprocess.PIPE)
	popen.wait()
	output = popen.stdout.read()
	print output
        # subscriptions
        #rospy.Subscriber("/grudsby/arduino_response", ArduinoResponse, self.responseCallback)
        
        #self.odomPub = rospy.Publisher("/grudsby/odometry", Odometry, queue_size=10)

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
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
