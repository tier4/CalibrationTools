
cmake_minimum_required(VERSION 3.5)
project(extrinsic_mapping_based_calibrator)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(autoware_cmake REQUIRED)
find_package(OpenMP)

autoware_package()

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


ament_export_include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# COMPILE THE SOURCE
# ========================================================================
ament_auto_add_executable(extrinsic_mapping_based_calibrator
  src/extrinsic_mapping_based_calibrator.cpp
  src/main.cpp
)

target_link_libraries(extrinsic_mapping_based_calibrator
)

add_definitions(${PCL_DEFINITIONS})

function(get_all_targets var)
    set(targets)
    get_all_targets_recursive(targets ${CMAKE_CURRENT_SOURCE_DIR})
    set(${var} ${targets} PARENT_SCOPE)
endfunction()

macro(get_all_targets_recursive targets dir)
    get_property(subdirectories DIRECTORY ${dir} PROPERTY SUBDIRECTORIES)
    foreach(subdir ${subdirectories})
        get_all_targets_recursive(${targets} ${subdir})
    endforeach()

    get_property(current_targets DIRECTORY ${dir} PROPERTY BUILDSYSTEM_TARGETS)
    list(APPEND ${targets} ${current_targets})
endmacro()

get_all_targets(all_targets)
message("All targets: ${all_targets}")

# Remove entry
get_target_property(target_options extrinsic_mapping_based_calibrator COMPILE_OPTIONS)
list(REMOVE_ITEM target_options "-pedantic")
list(REMOVE_ITEM target_options "-error")
list(REMOVE_ITEM target_options "-O3")
list(REMOVE_ITEM target_options "-march=native")
set_property(TARGET extrinsic_mapping_based_calibrator PROPERTY COMPILE_OPTIONS ${target_options})

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  rviz
)
