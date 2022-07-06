cmake_minimum_required(VERSION 3.5)
project(calibration_adapter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(pacmod_calibration_adapter
  src/pacmod_calibration_adapter_node.cpp
  src/calibration_adapter_node_base.cpp
)
ament_target_dependencies(pacmod_calibration_adapter)

ament_auto_add_executable(calibration_adapter
  src/calibration_adapter_node.cpp
  src/calibration_adapter_node_base.cpp
)
ament_target_dependencies(calibration_adapter)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
