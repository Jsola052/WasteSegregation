cmake_minimum_required(VERSION 3.5)
project(camera_cpp_pkg)
cmake_policy(SET CMP0074 NEW)  # Fix for PCL_ROOT warning

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(geometry_msgs)

# Executable for the image publisher node
add_executable(green_ball_filter src/green_ball_filter.cpp)
ament_target_dependencies(green_ball_filter rclcpp sensor_msgs pcl_ros PCL geometry_msgs)

install(TARGETS
  green_ball_filter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
