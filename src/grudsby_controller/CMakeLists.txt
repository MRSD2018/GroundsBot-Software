cmake_minimum_required(VERSION 2.8.3)
project(grudsby_controller)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  controller_manager_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#    FILES
# )

## Generate added messages and services with any dependencies listed here



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

