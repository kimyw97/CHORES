cmake_minimum_required(VERSION 3.8)
project(my_robot_arm_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
include_directories(/usr/local/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc/serial)
include_directories(/usr/include)

link_directories(/usr/local/lib)

# 👉 실행 노드 추가
add_executable(arm_control_node
src/arm_control_node.cpp
)
add_executable(arm_control_pub_node
  src/arm_control_pub_node.cpp
)

target_include_directories(arm_control_node PRIVATE inc impl)
target_link_libraries(arm_control_node serial)


install(TARGETS arm_control_node DESTINATION lib/${PROJECT_NAME})
ament_target_dependencies(arm_control_node rclcpp std_msgs)

ament_target_dependencies(arm_control_pub_node
  rclcpp
  std_msgs
)
install(TARGETS
  arm_control_node
  arm_control_pub_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


