cmake_minimum_required(VERSION 2.8.3)
project(grudsby_costmap)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  DIRECTORY msg
  FILES
  Throttle.msg)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs
  )
  #DEPENDS Boost)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h")
