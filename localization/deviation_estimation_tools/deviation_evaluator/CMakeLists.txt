cmake_minimum_required(VERSION 3.5)
project(deviation_evaluator)

find_package(autoware_cmake REQUIRED)
autoware_package()

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

ament_auto_add_executable(deviation_evaluator
  src/deviation_evaluator_node.cpp
  src/deviation_evaluator.cpp
)
ament_target_dependencies(deviation_evaluator)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(deviation_evaluator-test test/test_deviation_evaluator.test
#     test/src/test_deviation_evaluator.cpp
#     src/deviation_evaluator.cpp
#   )
#   target_include_directories(deviation_evaluator-test
#     PRIVATE
#     include
#   )
#   ament_target_dependencies(deviation_evaluator-test geometry_msgs rclcpp tf2 tf2_ros)
# endif()

install(PROGRAMS scripts/deviation_evaluation_visualizer.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  rviz
  config
)
