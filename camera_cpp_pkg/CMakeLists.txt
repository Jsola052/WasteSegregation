cmake_minimum_required(VERSION 3.5)

# Ensure find_package honors *_ROOT variables for PCL, etc.
cmake_policy(SET CMP0074 NEW)

project(camera_cpp_pkg)

find_package(ament_cmake       REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(sensor_msgs       REQUIRED)
find_package(pcl_ros           REQUIRED)
find_package(PCL               REQUIRED)
find_package(pcl_conversions   REQUIRED)
find_package(geometry_msgs     REQUIRED)
find_package(Eigen3            REQUIRED)
find_package(ur_interface      REQUIRED)
find_package(std_msgs          REQUIRED)
find_package(cv_bridge         REQUIRED)
find_package(OpenCV            REQUIRED)
find_package(CURL              REQUIRED)            # for libcurl
find_package(nlohmann_json     REQUIRED)            # for JSON parsing

include_directories(
  ${PCL_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)

# —————————————————————————————
# green_ball_filter node
add_executable(green_ball_filter
  src/green_ball_filter.cpp
)
ament_target_dependencies(green_ball_filter
  rclcpp
  sensor_msgs
  pcl_ros
  PCL
  geometry_msgs
  Eigen3
  ur_interface
  std_msgs
)
target_link_libraries(green_ball_filter
  ${PCL_LIBRARIES}
)

# —————————————————————————————
# detect_cans_publisher node
add_executable(detect_cans_publisher
  src/detect_cans_publisher.cpp
)
ament_target_dependencies(detect_cans_publisher
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  PCL
  pcl_conversions
)
target_link_libraries(detect_cans_publisher
  ${PCL_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  CURL::libcurl
  nlohmann_json::nlohmann_json
)

# —————————————————————————————
install(TARGETS
  green_ball_filter
  detect_cans_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
