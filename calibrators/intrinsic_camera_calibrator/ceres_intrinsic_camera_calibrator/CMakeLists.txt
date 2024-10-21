
cmake_minimum_required(VERSION 3.5)
project(ceres_intrinsic_camera_calibrator)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(autoware_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

# Find python before pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

#add_subdirectory(pybind11)

autoware_package()

# These need to be called after autoware_package to avoid being overwritten
find_package(Boost REQUIRED COMPONENTS system serialization filesystem)

# Optimizer as a library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ceres_camera_intrinsics_optimizer.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS})

ament_export_include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  )

# Test

# COMPILE THE SOURCE
#========================================================================
ament_auto_add_executable(${PROJECT_NAME}_test
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_test
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  ${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

#ament_python_install_package(${PROJECT_NAME}_py)

pybind11_add_module(${PROJECT_NAME}_py src/ceres_intrinsic_camera_calibrator_py.cpp)

target_include_directories(${PROJECT_NAME}_py PRIVATE include ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_py PRIVATE
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  ${PROJECT_NAME}
)

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
target_compile_definitions(${PROJECT_NAME}_py
                           PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})


message("====PYTHON_INSTALL_DIR=${PYTHON_INSTALL_DIR}")

install(
  TARGETS ${PROJECT_NAME}_py
  #DESTINATION "${PYTHON_INSTALL_DIR}"
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

#install(PROGRAMS
#  scripts/calibrator_ui_node.py
#  DESTINATION lib/${PROJECT_NAME}
#)

ament_export_dependencies(ament_cmake_python)

ament_auto_package()
