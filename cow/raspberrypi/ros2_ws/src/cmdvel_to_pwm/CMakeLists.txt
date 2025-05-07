cmake_minimum_required(VERSION 3.5)
project(cmdvel_to_pwm)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

# Launch 디렉토리 설치
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Config 디렉토리 설치
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Map 디렉토리 설치 (✅ 수정됨)
install(
  DIRECTORY map
  DESTINATION share/${PROJECT_NAME}
)

# 노드 실행 파일 설치
install(PROGRAMS
  cmdvel_to_pwm/cmdvel_to_pwm_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 파이썬 패키지 설치
ament_python_install_package(${PROJECT_NAME})

ament_package()
