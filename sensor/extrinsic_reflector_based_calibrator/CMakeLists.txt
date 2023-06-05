
cmake_minimum_required(VERSION 3.5)
project(extrinsic_reflector_based_calibrator)

find_package(autoware_cmake REQUIRED)

autoware_package()

ament_export_include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

  # COMPILE THE SOURCE
#========================================================================
ament_auto_add_executable(extrinsic_reflector_based_calibrator
  src/extrinsic_reflector_based_calibrator.cpp
  src/main.cpp
)

target_link_libraries(extrinsic_reflector_based_calibrator
  ${OpenCV_LIBS}
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    rviz
)
