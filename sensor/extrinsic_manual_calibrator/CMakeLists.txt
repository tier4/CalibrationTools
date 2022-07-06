cmake_minimum_required(VERSION 3.8)
project(extrinsic_manual_calibrator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(extrinsic_manual_calibrator
  src/extrinsic_manual_calibrator_node.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
