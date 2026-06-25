cmake_minimum_required(VERSION 3.8)
project(arm_control)

# C++17 표준 사용
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ==========================================================
# 1. 필요한 모든 패키지를 여기에 선언합니다.
# ==========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(gcode_interpreter REQUIRED)
# ==========================================================
# 2. 모든 실행 파일들을 정의합니다.
# ==========================================================
add_executable(joystick_node_92 src/joystick_node_92.cpp)
add_executable(joystick_to_rmd_control_node src/joystick_to_rmd_control_node.cpp)
# add_executable(joystick_node_fk src/joystick_node_fk.cpp) # 주석 처리
add_executable(inverse_kinematics_node src/inverse_kinematics_node.cpp)

# ==========================================================
# 3. 각 실행 파일의 의존성을 설정합니다.
# ==========================================================
ament_target_dependencies(joystick_node_92
  rclcpp
  std_msgs
  sensor_msgs
  gcode_interpreter
)

# joystick_node_aq 의존성을 새 이름으로 변경
ament_target_dependencies(joystick_to_rmd_control_node
  rclcpp
  sensor_msgs
  std_msgs
  gcode_interpreter
)

# joystick_node_fk 의존성 주석 처리
# ament_target_dependencies(joystick_node_fk
#   rclcpp
#   std_msgs
#   geometry_msgs
# )
# target_link_libraries(joystick_node_fk Eigen3::Eigen)

ament_target_dependencies(inverse_kinematics_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  custom_msgs
  gcode_interpreter
)
target_link_libraries(inverse_kinematics_node Eigen3::Eigen)

# ==========================================================
# 4. 생성된 실행 파일과 리소스 파일을 설치합니다.
# ==========================================================
install(TARGETS
  joystick_node_92
  # joystick_node_aq를 새 이름으로 변경
  joystick_to_rmd_control_node
  # joystick_node_fk # 설치 목록에서 제거
  inverse_kinematics_node
  DESTINATION lib/${PROJECT_NAME}
)

# launch 디렉토리가 있다면 설치
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# urdf 디렉토리가 있다면 설치
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/urdf")
  install(DIRECTORY
    urdf
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# rviz 디렉토리가 있다면 설치
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(DIRECTORY
    rviz
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# ==========================================================
# 5. ament_package()로 마무리합니다.
# ==========================================================
ament_package()

install(PROGRAMS
  scripts/moveit_to_rmd_bridge.py
  scripts/real_joint_state_mapper.py
  DESTINATION lib/${PROJECT_NAME}
)
