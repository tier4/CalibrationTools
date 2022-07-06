cmake_minimum_required(VERSION 3.5)
project(tunable_static_tf_broadcaster)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(tunable_static_tf_broadcaster_node_component SHARED
  src/tunable_static_tf_broadcaster_node.cpp
)

rclcpp_components_register_node(tunable_static_tf_broadcaster_node_component
  PLUGIN "tunable_static_tf_broadcaster::TunableStaticTfBroadcasterNode"
  EXECUTABLE tunable_static_tf_broadcaster_node
)

## Package
ament_auto_package(
  INSTALL_TO_SHARE
    params
    launch
)
