cmake_minimum_required(VERSION 3.8)
project(arctos_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
