cmake_minimum_required(VERSION 2.8.3)
project(differential_drive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

# Generate messages in the 'msg' folder


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



