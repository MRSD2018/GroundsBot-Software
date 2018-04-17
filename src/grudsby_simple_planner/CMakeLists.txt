cmake_minimum_required(VERSION 2.8.3)
project(grudsby_simple_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  geometry_msgs
  message_generation
  nav_msgs
)

# Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
  FILES
  SimplePlannerDebug.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES grudsby_simple_planner
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime 
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME} src/grudsby_simple_planner.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(grudsby_simple_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS grudsby_simple_planner grudsby_simple_planner
  # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
   PATTERN ".svn" EXCLUDE
)

