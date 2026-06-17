cmake_minimum_required(VERSION 3.8)
project(arm_only_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
