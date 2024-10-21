cmake_minimum_required(VERSION 3.5)
project(tier4_calibration_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CalibrationPoints.msg"
  "msg/CalibrationResult.msg"
  "msg/Files.msg"
  "srv/Empty.srv"
  "srv/FilesSrv.srv"
  "srv/FilesListSrv.srv"
  "srv/CalibrationDatabase.srv"
  "srv/Frame.srv"
  "srv/IntrinsicsOptimizer.srv"
  "srv/ExtrinsicCalibrator.srv"
  DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
