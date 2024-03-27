
cmake_minimum_required(VERSION 3.5)
project(lidar_to_lidar_2d_calibrator)

find_package(autoware_cmake REQUIRED)
find_package(OpenCV REQUIRED) # TODO: consider removing this one later

autoware_package()

ament_export_include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# COMPILE THE SOURCE
# ========================================================================
ament_auto_add_executable(lidar_to_lidar_2d_calibrator
  src/lidar_to_lidar_2d_calibrator.cpp
  src/main.cpp
)

target_link_libraries(lidar_to_lidar_2d_calibrator
  ${OpenCV_LIBS}
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  rviz
)
