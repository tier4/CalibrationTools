cmake_minimum_required(VERSION 3.8)
project(extrinsic_map_based_calibrator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(extrinsic_map_based_calibrator
  src/extrinsic_map_based_calibrator.cpp
  src/extrinsic_map_based_preprocessing.cpp
  src/grid_search_matching.cpp)
ament_target_dependencies(extrinsic_map_based_calibrator)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
