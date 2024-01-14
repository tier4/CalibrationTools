cmake_minimum_required(VERSION 3.8)
project(point_cloud_accumulator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(point_cloud_accumulator
  src/point_cloud_accumulator.cpp)
ament_target_dependencies(point_cloud_accumulator)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
