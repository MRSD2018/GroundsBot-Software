cmake_minimum_required(VERSION 2.8.3)
project(grudsby_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  geometry_msgs
  message_generation
  sensor_msgs
)






include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


