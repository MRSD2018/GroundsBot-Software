cmake_minimum_required(VERSION 2.8.3)
project(grudsby_rfcomms)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
