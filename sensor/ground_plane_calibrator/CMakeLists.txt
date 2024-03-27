
cmake_minimum_required(VERSION 3.5)
project(ground_plane_calibrator)

find_package(autoware_cmake REQUIRED)

autoware_package()

ament_export_include_directories(
  include
)

  # COMPILE THE SOURCE
#========================================================================
ament_auto_add_executable(ground_plane_calibrator
  src/ground_plane_calibrator.cpp
  src/main.cpp
)

target_link_libraries(ground_plane_calibrator
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    rviz
)
