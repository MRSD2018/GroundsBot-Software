cmake_minimum_required(VERSION 2.8.3)
project(grudsby_gui)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  rqt_gui
  rqt_gui_py
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/grudsby_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
