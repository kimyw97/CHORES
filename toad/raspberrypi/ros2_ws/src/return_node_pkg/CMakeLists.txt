cmake_minimum_required(VERSION 3.8)
project(motion_mirror_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(motion_interfaces REQUIRED)  # 너가 만든 SerialCmd.msg 포함한 메시지 패키지

add_executable(motion_mirror_test src/motion_mirror_test.cpp)

ament_target_dependencies(motion_mirror_test
  rclcpp
  std_msgs
  motion_interfaces
)

install(TARGETS motion_mirror_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
