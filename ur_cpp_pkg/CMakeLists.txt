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
find_package(ur_interface)
find_package(nlohmann_json REQUIRED)

find_package(pinocchio REQUIRED)

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
  tf2_geometry_msgs
  ur_interface)
target_link_libraries(ball_pickup_demo ${UR_RTDE_LIBRARY})

add_executable(vial_pickup_node src/vial_pickup_node.cpp)
ament_target_dependencies(vial_pickup_node 
  rclcpp 
  std_msgs 
  nlohmann_json 
  tf2 
  tf2_ros 
  tf2_geometry_msgs)
target_link_libraries(vial_pickup_node ${UR_RTDE_LIBRARY})

add_executable(gripper_subscriber src/gripper_subscriber.cpp)
ament_target_dependencies(gripper_subscriber 
  rclcpp 
  std_msgs)
target_link_libraries(gripper_subscriber ${UR_RTDE_LIBRARY})

add_executable(gripper_test src/gripper_test.cpp)
ament_target_dependencies(gripper_test rclcpp)
target_link_libraries(gripper_test ${UR_RTDE_LIBRARY})

add_executable(ur_basic_control_node src/ur_basic_control_node.cpp)
ament_target_dependencies(ur_basic_control_node rclcpp)

target_link_libraries(ur_basic_control_node
  ${UR_RTDE_LIBRARY}
)

add_executable(ur_pinocchio_planner_node src/ur_pinocchio_planner.cpp)
ament_target_dependencies(ur_pinocchio_planner_node rclcpp)

target_link_libraries(ur_pinocchio_planner_node
  pinocchio::pinocchio
  ${UR_RTDE_LIBRARY}
)


# Install all executables
install(TARGETS
  get_tool_subscriber
  tool_publisher
  tool_changer_success_subscriber
  ball_pickup_demo
  vial_pickup_node
  gripper_subscriber
  gripper_test  
  ur_pinocchio_planner_node
  ur_basic_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
