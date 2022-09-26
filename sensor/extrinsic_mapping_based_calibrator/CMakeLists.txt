
cmake_minimum_required(VERSION 3.5)
project(extrinsic_mapping_based_calibrator)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(autoware_cmake REQUIRED)
find_package(OpenMP)

autoware_package()

# These need to be called after autoware_package to avoid being overwritten
find_package(Boost REQUIRED COMPONENTS system serialization filesystem)

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

ament_export_include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# COMPILE THE SOURCE
# ========================================================================

ament_auto_add_executable(extrinsic_mapping_based_calibrator
  src/filters/dynamics_filter.cpp
  src/filters/lost_state_filter.cpp
  src/filters/best_frames_filter.cpp
  src/filters/object_detection_filter.cpp
  src/calibration_mapper.cpp
  src/extrinsic_mapping_based_calibrator.cpp
  src/lidar_calibrator.cpp
  src/main.cpp
  src/utils.cpp
)

target_link_libraries(extrinsic_mapping_based_calibrator
  ${Boost_LIBRARIES})

add_definitions(${PCL_DEFINITIONS})

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  rviz
)
