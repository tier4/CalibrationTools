cmake_minimum_required(VERSION 3.5)
project(time_delay_estimator)

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

ament_auto_add_executable(time_delay_estimator
  src/time_delay_estimator_node.cpp
  src/time_delay_estimator.cpp
  src/data_processor.cpp
  src/estimator.cpp
  src/main.cpp)
ament_target_dependencies(time_delay_estimator)

ament_auto_add_executable(general_time_delay_estimator
  src/general_time_delay_estimator_node.cpp
  src/time_delay_estimator.cpp
  src/data_processor.cpp
  src/estimator.cpp
  src/main.cpp)
ament_target_dependencies(general_time_delay_estimator)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS scripts/correlation_visualizer.py DESTINATION lib/${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE
  launch
  config)
