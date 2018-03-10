cmake_minimum_required(VERSION 2.8.3)
project(grudsby_sweeping)

add_compile_options(-std=c++11)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs 
  nav_msgs
  rospy
  geodesy
  tf
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  SimpleLatLng.msg
  MowingPlan.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/parsePath.cpp
  src/segment.cpp
  src/slice.cpp
  src/boustrophedon.cpp
  src/sign.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_planner src/boustrophedon_node.cpp)

add_dependencies(${PROJECT_NAME}_planner ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_planner
  ${catkin_LIBRARIES}
  grudsby_sweeping
  curl
  png
)

