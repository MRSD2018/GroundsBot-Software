cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(grudsby_gazebo)

# Find Gazebo
find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS roslaunch)


include_directories(
 #include
  ${catkin_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/include
)


catkin_package(
  CATKIN_DEPENDS
)

# Build our plugin
add_library(grudsby_plugin SHARED src/grudsby_plugin.cpp)
target_link_libraries(grudsby_plugin ${GAZEBO_libraries} )

# Build the stand-alone test program
add_executable(vel src/vel.cc)

if (${gazebo_VERSION_MAJOR} LESS 6)
  # These two
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  target_link_libraries(vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else()
  target_link_libraries(vel ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
endif()
