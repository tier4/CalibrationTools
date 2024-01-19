cmake_minimum_required(VERSION 3.5)
project(tier4_calibration_views)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(autoware_cmake REQUIRED)

autoware_package()
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/image_view_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_package()
