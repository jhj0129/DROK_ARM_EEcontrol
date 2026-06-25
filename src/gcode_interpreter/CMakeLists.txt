cmake_minimum_required(VERSION 3.8)
project(gcode_interpreter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 필요한 모든 패키지를 먼저 찾습니다.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# custom_msgs 의존성은 현재 코드에서는 필요 없으므로 제거해도 됩니다.
# 만약 gcode_interpreter 노드가 custom_msgs의 메시지를 직접 사용한다면 남겨둡니다.
# find_package(custom_msgs REQUIRED) 

# 2. 이 패키지 자체의 메시지를 생성합니다.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Param.msg"
  "msg/GcodeCommand.msg"
)

# 3. 실행 파일을 정의합니다.
add_executable(gcode_interpreter_node src/gcode_interpreter_node.cpp)

# 4. ★★★ 실행 파일에 필요한 라이브러리들을 연결(링크)합니다. ★★★
# 이 부분이 가장 중요하고, 이전 답변에서 틀렸던 부분입니다.
ament_target_dependencies(gcode_interpreter_node
  rclcpp
  custom_msgs
  tf2_geometry_msgs
)

# ★★★ 메시지 생성과 실행 파일 빌드를 연결하는 부분 ★★★
# "gcode_interpreter_node를 만들기 전에, 이 패키지의 메시지 생성이 반드시 끝나야 한다"
# 라고 명시적으로 선언합니다.
rosidl_get_typesupport_target(gcode_interpreter_cpp
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gcode_interpreter_node "${gcode_interpreter_cpp}")


# 5. 설치 규칙을 정의합니다.
install(
  TARGETS gcode_interpreter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()