cmake_minimum_required(VERSION 3.5)
project(ur_cpp_pkg)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tool_changer_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Manually specify the paths to ur_rtde
set(UR_RTDE_DIR "/home/robotics/Projects/lib_/ur_rtde")
set(UR_RTDE_INCLUDE_DIR "${UR_RTDE_DIR}/include")
set(UR_RTDE_LIBRARY "${UR_RTDE_DIR}/build/librtde.so")

# Include directories
include_directories(
  ${UR_RTDE_INCLUDE_DIR}
)

# Debugging output to verify paths
message(STATUS "UR_RTDE_INCLUDE_DIR: ${UR_RTDE_INCLUDE_DIR}")
message(STATUS "UR_RTDE_LIBRARY: ${UR_RTDE_LIBRARY}")

# Add executables
add_executable(get_tool_subscriber src/get_tool_subscriber.cpp)
ament_target_dependencies(get_tool_subscriber rclcpp std_msgs tool_changer_interface)
target_link_libraries(get_tool_subscriber ${UR_RTDE_LIBRARY})

add_executable(tool_publisher src/tool_publisher.cpp)
ament_target_dependencies(tool_publisher rclcpp std_msgs)

add_executable(tool_changer_success_subscriber src/tool_changer_success_subscriber.cpp)
ament_target_dependencies(tool_changer_success_subscriber rclcpp std_msgs)

add_executable(ball_pickup_demo src/ball_pickup_demo.cpp)
ament_target_dependencies(ball_pickup_demo 
  rclcpp 
  geometry_msgs 
  std_msgs 
  visualization_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs)
target_link_libraries(ball_pickup_demo ${UR_RTDE_LIBRARY})

# Install the targets
install(TARGETS
  get_tool_subscriber
  tool_publisher
  tool_changer_success_subscriber
  ball_pickup_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
