cmake_minimum_required(VERSION 3.5)
project(toad_auto_drive)

# 필수 패키지들
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(OpenCV REQUIRED)  # 🔹 OpenCV 추가
find_package(rosidl_default_generators REQUIRED)

# 메시지 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ToadDriveMsg.msg"
  "msg/TrashInfo.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# 퍼블리셔 노드
add_executable(toad_auto_drive_pub src/toad_auto_drive_pub.cpp)
ament_target_dependencies(toad_auto_drive_pub
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  serial
  OpenCV         # 🔹 OpenCV 의존성 추가
)

# 서브스크라이버 노드
add_executable(auto_drive_sub_node src/toad_auto_drive_sub.cpp)
ament_target_dependencies(auto_drive_sub_node
  rclcpp
  std_msgs
  serial
)

# 타입서포트
rosidl_get_typesupport_target(
  TYPESUPPORT_TARGET
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)

# 🔹 OpenCV 링크 추가
target_link_libraries(toad_auto_drive_pub
   ${TYPESUPPORT_TARGET}
   ${OpenCV_LIBS}
)
target_link_libraries(auto_drive_sub_node
   ${TYPESUPPORT_TARGET}
)

# 설치 지시
install(
  TARGETS
    toad_auto_drive_pub
    auto_drive_sub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
