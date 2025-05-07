cmake_minimum_required(VERSION 3.5)
project(desk_pickup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1) .msg 파일 생성 (add_executable 이전)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ToadDriveMsg.msg"
  DEPENDENCIES std_msgs  # 만약 ToadDriveMsg.msg 안에 geometry_msgs 타입을 쓰셨다면 여기에 geometry_msgs 도 추가
)

ament_export_dependencies(rosidl_default_runtime)

# 2) 퍼블리셔 노드
add_executable(toad_auto_drive_pub src/toad_auto_drive_pub.cpp)
ament_target_dependencies(toad_auto_drive_pub
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  serial
)

# 3) 서브스크라이버 노드 (여기서도 add_executable 먼저!)
add_executable(auto_drive_sub_node src/toad_auto_drive_sub.cpp)
ament_target_dependencies(auto_drive_sub_node
  rclcpp
  std_msgs
  serial
)

# 올바른 호출 순서
rosidl_get_typesupport_target(
  TYPESUPPORT_TARGET       # 1) 결과를 받을 변수
  ${PROJECT_NAME}          # 2) rosidl_generate_interfaces() 에 넘긴 패키지명
  rosidl_typesupport_cpp   # 3) 타입 서포트 식별자
)


target_link_libraries(toad_auto_drive_pub
   ${TYPESUPPORT_TARGET}
)
target_link_libraries(auto_drive_sub_node
   ${TYPESUPPORT_TARGET}
)

# 5) 설치 지시
install(
  TARGETS
    toad_auto_drive_pub
    auto_drive_sub_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
