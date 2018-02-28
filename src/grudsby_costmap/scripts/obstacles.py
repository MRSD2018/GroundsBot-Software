#!/usr/bin/env python
################################################################################
## Process a costmap, determine closest obstacle, send throttle command ########
#### accordingly ###############################################################
################################################################################

import rospy
import roslib
import math
import numpy as np

from grudsby_costmap.msg import Throttle
from nav_msgs.msg import OccupancyGrid


################################################################################
## Define map ranges and corresponding throttle values #########################
################################################################################

class Obstacle_Range:
  def __init__( self , name = '' , range_min = 0, range_max = 0 , throttle = 0):
    self.name      = name 
    self.range_min = range_min 
    self.range_max = range_max 
    self.throttle  = throttle

throttle_ranges = [ \
  Obstacle_Range( name = 'dead', range_min = 0 , range_max = 1 , throttle = 0.0 ) ,
  Obstacle_Range( name = 'near', range_min = 1 , range_max = 2 , throttle = 0.2 ) ,
  Obstacle_Range( name = 'mid' , range_min = 2 , range_max = 4 , throttle = 0.5 ) ,
  Obstacle_Range( name = 'far' , range_min = 4 , range_max = 10 , throttle = 0.8 ) ]

################################################################################
## throttle_from_occupancy_grid ################################################
##### Check costmap for obstacles based on predefined ranges ###################
##### Return throttling value 0 - 1 (0: stop , 1:no effect ) ###################
################################################################################

def throttle_from_occupancy_grid ( grid , grid_resolution , ranges ):
  #assert all ( [ type ( r ) == Obstacle_Detection for r in ranges ] )
  throttle = 1
  for r in ranges:
    low  = int ( np.floor ( r.range_min / grid_resolution ) )
    high = int ( np.ceil  ( r.range_max / grid_resolution ) )
    try : 
      if np.any ( grid [ low : high , : ] ):
        throttle = min ( throttle , r.throttle )
        #print 'obstacle detected in {}'.format(r.name )
    except IndexError:
      print 'occupancy grid index out of range during obstacle check'
  return throttle

################################################################################
## Look for costmaps , process costmaps, publish throttling message ############
################################################################################

class Obstacle_Detection:
  def __init__(self , throttle_ranges):

    self.throttle_ranges = throttle_ranges

    rospy.init_node ("obstacle_detection")
    self.nodename = rospy.get_name()
    rospy.loginfo("-I- %s started" % self.nodename)
    self.rate = 200
    self.throttlePub = rospy.Publisher("/velocity_throttle", Throttle, queue_size=10)
    rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.costmap_callback)

  def spin(self):
    r = rospy.Rate(self.rate)
    while not rospy.is_shutdown():
      self.update()
      r.sleep()

  def update(self):
    pass

  def costmap_callback(self, msg):
    res     = msg.info.resolution # m / cell
    w       = msg.info.width      # cells
    h       = msg.info.height     # cells
    costmap = np.array ( msg.data )
    costmap = costmap.reshape ( h , w )

    throttle_msg        = Throttle()
    throttle_msg.header = msg.header # ?
    throttle_msg.enable = True

    throttle_msg.max_speed_percentage = \
      throttle_from_occupancy_grid ( costmap , res , self.throttle_ranges )

    self.throttlePub.publish( throttle_msg )

if __name__ == '__main__':
  try:
    obstacles = Obstacle_Detection (throttle_ranges)
    obstacles.spin()
  except rospy.ROSInterruptException:
    pass
