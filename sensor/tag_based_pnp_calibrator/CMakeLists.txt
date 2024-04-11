
cmake_minimum_required(VERSION 3.5)
project(tag_based_pnp_calibrator)

find_package(autoware_cmake REQUIRED)
find_package(OpenCV REQUIRED)

autoware_package()

ament_export_include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

ament_auto_add_executable(tag_based_pnp_calibrator
  src/brute_force_matcher.cpp
  src/calibration_estimator.cpp
  src/tag_based_pnp_calibrator.cpp
  src/tag_calibrator_visualizer.cpp
  src/main.cpp
)

target_link_libraries(tag_based_pnp_calibrator
  ${OpenCV_LIBS}
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    rviz
)
