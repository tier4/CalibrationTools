cmake_minimum_required(VERSION 3.5)
project(deviation_estimator)

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

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

ament_auto_add_executable(deviation_estimator
  src/deviation_estimator_node.cpp
  src/deviation_estimator.cpp
)
ament_target_dependencies(deviation_estimator)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(deviation_estimator-test test/test_deviation_estimator.test
#     test/src/test_deviation_estimator.cpp
#     src/deviation_estimator.cpp
#   )
#   target_include_directories(deviation_estimator-test
#     PRIVATE
#     include
#   )
#   ament_target_dependencies(deviation_estimator-test geometry_msgs rclcpp tf2 tf2_ros)
# endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
