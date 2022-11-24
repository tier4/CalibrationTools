cmake_minimum_required(VERSION 3.5)
project(deviation_estimator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(deviation_estimator
  src/deviation_estimator_node.cpp
  src/deviation_estimator.cpp
  src/utils.cpp
  src/gyro_bias_module.cpp
  src/velocity_coef_module.cpp
)
ament_target_dependencies(deviation_estimator)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
