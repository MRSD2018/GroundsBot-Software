cmake_minimum_required(VERSION 2.8.3)
project(tegra_stereo)

find_package(catkin REQUIRED
  COMPONENTS roscpp nodelet
  cv_bridge image_geometry image_proc image_transport
  message_filters sensor_msgs stereo_msgs
  camera_calibration_parsers camera_info_manager)
find_package(Boost REQUIRED COMPONENTS thread)

find_package(OpenCV REQUIRED)

find_package(PkgConfig)
pkg_check_modules(YAML_CPP yaml-cpp)
if(${YAML_CPP_VERSION} VERSION_GREATER 0.5)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()

# C++11 is required
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_STD_CXX11)
if (COMPILER_SUPPORTS_STD_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else ()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()

# check for arm processor
exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" ) # i.e. Jetson TX1 (I think native would work in TX1 as well as it will pick the host arch...)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -march=armv8-a")
else() # Ubuntu Desktop
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -march=native")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tegra_stereo
  CATKIN_DEPENDS roscpp nodelet
  cv_bridge image_geometry image_proc image_transport
  message_filters sensor_msgs stereo_msgs
  camera_calibration_parsers camera_info_manager
  DEPENDS OpenCV
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})

find_package(CUDA REQUIRED)

set(CUDA_PROPAGATE_HOST_FLAGS OFF)
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11 -w -O3 -lineinfo")

#virtual architectures based on this post: http://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/
#set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode=arch=compute_20,code=sm_20")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode=arch=compute_30,code=sm_30")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode=arch=compute_50,code=sm_50")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode=arch=compute_52,code=sm_52")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode=arch=compute_52,code=compute_52")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode=arch=compute_60,code=sm_60")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode=arch=compute_61,code=sm_61")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode=arch=compute_62,code=sm_62")
# building for phisical architecture sm_53 (i.e. TX1) , other will use JIT
set(CUDA_ARCH "-arch=sm_62")
set(LIBSGM 
	src/median_filter.cu  
	src/hamming_cost.cu 
	src/disparity_method.cu
	src/debug.cu 
	src/costs.cu
)

cuda_add_library(tegra_stereo_proc src/tegra_stereo_proc.cpp ${LIBSGM})
target_link_libraries(tegra_stereo_proc ${CUDA_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})

################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files
    ${CMAKE_SOURCE_DIR}/*)
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})
################################################
