cmake_minimum_required(VERSION 3.8)
project(extrinsic_map_based_calibrator)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

ament_auto_add_executable(extrinsic_map_based_calibrator
  src/extrinsic_map_based_calibrator.cpp
  src/extrinsic_map_based_preprocessing.cpp
  src/grid_search_matching.cpp)
ament_target_dependencies(extrinsic_map_based_calibrator)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

if(OpenMP_CXX_FOUND)
  target_link_libraries(extrinsic_map_based_calibrator OpenMP::OpenMP_CXX)
else()
  message(WARNING "OpenMP not found")
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
