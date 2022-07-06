cmake_minimum_required(VERSION 3.5)
project(stop_accel_evaluator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(stop_accel_evaluator_node SHARED
  src/stop_accel_evaluator_node.cpp
)

rclcpp_components_register_node(stop_accel_evaluator_node
  PLUGIN "stop_accel_evaluator::StopAccelEvaluatorNode"
  EXECUTABLE stop_accel_evaluator
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
