cmake_minimum_required(VERSION 3.5)
project(ur16_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
