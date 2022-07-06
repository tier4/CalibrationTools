cmake_minimum_required(VERSION 3.8)
project(extrinsic_calibration_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(extrinsic_calibration_manager
  src/extrinsic_calibration_manager_node.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
