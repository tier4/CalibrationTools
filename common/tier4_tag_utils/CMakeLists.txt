
cmake_minimum_required(VERSION 3.5)
project(tier4_tag_utils)

find_package(autoware_cmake REQUIRED)
find_package(OpenCV REQUIRED)

autoware_package()

ament_auto_add_library(tier4_tag_utils_lib SHARED
  src/lidartag_hypothesis.cpp
  src/apriltag_hypothesis.cpp
)

target_link_libraries(tier4_tag_utils_lib
${OpenCV_LIBS}
)

target_include_directories(tier4_tag_utils_lib
  PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS})

  ament_export_include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
  )

ament_auto_add_library(apriltag_filter SHARED
  src/apriltag_filter.cpp
)

ament_auto_add_library(lidartag_filter SHARED
  src/lidartag_filter.cpp
)

target_link_libraries(apriltag_filter
  ${OpenCV_LIBS}
  tier4_tag_utils_lib
)

target_link_libraries(lidartag_filter
  ${OpenCV_LIBS}
  tier4_tag_utils_lib
)

rclcpp_components_register_node(apriltag_filter
  PLUGIN "tier4_tag_utils::ApriltagFilter"
  EXECUTABLE apriltag_filter_node
)

rclcpp_components_register_node(lidartag_filter
  PLUGIN "tier4_tag_utils::LidartagFilter"
  EXECUTABLE lidartag_filter_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
