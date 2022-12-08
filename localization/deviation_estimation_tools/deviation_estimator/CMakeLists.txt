cmake_minimum_required(VERSION 3.5)
project(deviation_estimator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(deviation_estimator_lib SHARED
  src/deviation_estimator_node.cpp
  src/deviation_estimator.cpp
  src/utils.cpp
  src/gyro_bias_module.cpp
  src/velocity_coef_module.cpp
  src/data_validation_module.cpp
  src/write_result_file.cpp
)

ament_auto_add_executable(deviation_estimator src/deviation_estimator_node.cpp)

target_compile_options(deviation_estimator PUBLIC -g -Wall -Wextra -Wpedantic -Werror)

target_link_libraries(deviation_estimator deviation_estimator_lib)
target_include_directories(deviation_estimator PUBLIC include)


if(BUILD_TESTING)
  function(add_testcase filepath)
    get_filename_component(filename ${filepath} NAME)
    string(REGEX REPLACE ".cpp" "" test_name ${filename})

    ament_add_gtest(${test_name} ${filepath})
    target_link_libraries("${test_name}" deviation_estimator_lib)
    ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
  endfunction()

  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_FILES
    test/test_gyro_stddev.cpp)

  foreach(filepath ${TEST_FILES})
    add_testcase(${filepath})
  endforeach()
endif()


ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
