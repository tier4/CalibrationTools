
cmake_minimum_required(VERSION 3.5)
project(marker_radar_lidar_calibrator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Ceres REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_auto_add_executable(marker_radar_lidar_calibrator
  src/marker_radar_lidar_calibrator.cpp
  src/transformation_estimator.cpp
  src/track.cpp
  src/main.cpp
)

target_link_libraries(marker_radar_lidar_calibrator
  ${CERES_LIBRARIES}
)

install(PROGRAMS
  scripts/calibrator_ui_node.py
  scripts/metrics_plotter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake_python)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    rviz
)
