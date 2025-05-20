cmake_minimum_required(VERSION 3.5)
project(trash_camera)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 메시지 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrashInfo.msg"
  DEPENDENCIES geometry_msgs
)

# 파이썬 실행 파일 설치
install(
  PROGRAMS
  trash_camera/trash_camera_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
