
cmake_minimum_required(VERSION 3.5)
project(extrinsic_tag_based_sfm_calibrator)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(apriltag REQUIRED)
find_package(autoware_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

autoware_package()

# These need to be called after autoware_package to avoid being overwritten
find_package(Boost REQUIRED COMPONENTS system serialization filesystem)

ament_python_install_package(${PROJECT_NAME})

ament_export_include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

  # COMPILE THE SOURCE
#========================================================================
ament_auto_add_executable(extrinsic_tag_based_sfm_calibrator
  src/ceres/calibration_problem.cpp
  src/intrinsics_calibration/intrinsics_calibrator.cpp
  src/intrinsics_calibration/apriltag_calibrator.cpp
  src/intrinsics_calibration/chessboard_calibrator.cpp
  src/calibration_scene_extractor.cpp
  src/extrinsic_tag_based_sfm_calibrator.cpp
  src/apriltag_detection.cpp
  src/apriltag_detector.cpp
  src/main.cpp
  src/math.cpp
  src/visualization.cpp
)

target_link_libraries(extrinsic_tag_based_sfm_calibrator
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  apriltag::apriltag
  ${CERES_LIBRARIES}
)

install(PROGRAMS
  scripts/calibrator_ui_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake_python)

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
    rviz
)
