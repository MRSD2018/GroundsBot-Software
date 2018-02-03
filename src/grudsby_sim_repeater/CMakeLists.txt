cmake_minimum_required(VERSION 2.8.3)
project(sim_repeater)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  sensor_msgs
  geometry_msgs
)

# Generate messages in the 'msg' folder
#add_message_files(
#   DIRECTORY msg
#   FILES
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
#)

catkin_package(
 CATKIN_DEPENDS sensor_msgs nav_msgs roscpp std_msgs rospy roscpp
 #DEPENDS system_lib
)



include_directories(
 #include
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

#add_executable(${PROJECT_NAME} src/temp_repeater.cpp)
#add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )
