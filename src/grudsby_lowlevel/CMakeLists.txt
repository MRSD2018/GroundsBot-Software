cmake_minimum_required(VERSION 2.8.3)
project(grudsby_lowlevel)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  geometry_msgs
  message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg
   FILES
   ArduinoVel.msg
   ArduinoResponse.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS message_runtime roscpp std_msgs rospy roscpp geometry_msgs
 #DEPENDS system_lib
)



include_directories(
 #include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(${PROJECT_NAME} src/grudsby_lowlevel.cpp)
#add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )
