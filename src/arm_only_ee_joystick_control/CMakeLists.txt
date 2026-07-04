cmake_minimum_required(VERSION 3.8)
project(arm_only_ee_joystick_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(ee_joy_plan_execute_node
  src/ee_joy_plan_execute_node.cpp
)

add_executable(ee_precision_pose_plan_node
  src/ee_precision_pose_plan_node.cpp
)

ament_target_dependencies(ee_joy_plan_execute_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  moveit_msgs
  moveit_ros_planning_interface
  tf2_geometry_msgs
)

ament_target_dependencies(ee_precision_pose_plan_node
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit_msgs
  moveit_ros_planning_interface
  tf2_geometry_msgs
)

install(TARGETS
  ee_joy_plan_execute_node
  ee_precision_pose_plan_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS
  scripts/camera_center_to_target_node
  scripts/vlm_mask_position_collector_node
  DESTINATION lib/${PROJECT_NAME}
)


install(PROGRAMS
  scripts/rs_py_rgbd_camera_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(camera_only_scan_once_cpp_node
  src/camera_only_scan_once_cpp_node.cpp
)

ament_target_dependencies(camera_only_scan_once_cpp_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

target_link_libraries(camera_only_scan_once_cpp_node
  ${OpenCV_LIBS}
)

install(TARGETS
  camera_only_scan_once_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)



add_executable(ee_grasp_pose_plan_node
  src/ee_grasp_pose_plan_node.cpp
)

ament_target_dependencies(ee_grasp_pose_plan_node
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit_msgs
  moveit_ros_planning_interface

  moveit_core)

install(TARGETS
  ee_grasp_pose_plan_node
  DESTINATION lib/${PROJECT_NAME}
)



add_executable(ee_obstacle_waypoint_plan_node
  src/ee_obstacle_waypoint_plan_node.cpp
)

ament_target_dependencies(ee_obstacle_waypoint_plan_node
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit_msgs
  shape_msgs
  moveit_core
  moveit_ros_planning_interface
)

install(TARGETS
  ee_obstacle_waypoint_plan_node
  DESTINATION lib/${PROJECT_NAME}
)

