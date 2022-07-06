cmake_minimum_required(VERSION 3.5)
project(vehicle_cmd_analyzer)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(vehicle_cmd_analyzer_node SHARED
  src/vehicle_cmd_analyzer.cpp
)

rclcpp_components_register_node(vehicle_cmd_analyzer_node
  PLUGIN "VehicleCmdAnalyzer"
  EXECUTABLE vehicle_cmd_analyzer
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
