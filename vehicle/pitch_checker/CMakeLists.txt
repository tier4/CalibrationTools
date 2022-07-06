cmake_minimum_required(VERSION 3.5)
project(pitch_checker)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(pitch_checker
  src/pitch_checker_node.cpp
  src/main.cpp
)
ament_target_dependencies(pitch_checker)

ament_auto_add_library(pitch_compare SHARED
  src/pitch_compare.cpp
  src/pitch_reader.cpp
)
ament_target_dependencies(pitch_compare)

install(
  PROGRAMS
    scripts/view_pitch.py
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
