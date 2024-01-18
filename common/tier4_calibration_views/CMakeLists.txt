cmake_minimum_required(VERSION 3.5)
project(tier4_calibration_views)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(autoware_cmake REQUIRED)
#find_package(sensor_msgs REQUIRED)
#find_package(nav_msgs REQUIRED)
#find_package(geometry_msgs REQUIRED)

autoware_package()
ament_python_install_package(${PROJECT_NAME})

#if(BUILD_TESTING)
#  find_package(rclpy REQUIRED)
#  find_package(ament_cmake_nose REQUIRED)

#  ament_add_nose_test(pointclouds test/test_pointclouds.py)
#  ament_add_nose_test(images test/test_images.py)
#  ament_add_nose_test(occupancygrids test/test_occupancygrids.py)
#  ament_add_nose_test(geometry test/test_geometry.py)
#  ament_add_nose_test(quaternions test/test_quat.py)
#endif()

install(PROGRAMS
  scripts/image_view_node.py
  DESTINATION lib/${PROJECT_NAME}
)

##############
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_package()
