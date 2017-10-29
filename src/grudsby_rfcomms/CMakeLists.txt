cmake_minimum_required(VERSION 2.8.3)
project(grudsby_rfcomms)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
# add_message_files(
#    FILES
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
