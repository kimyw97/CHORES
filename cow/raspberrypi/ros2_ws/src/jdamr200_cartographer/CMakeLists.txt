cmake_minimum_required(VERSION 3.8)
project(jdamr200_cartographer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cartographer_ros REQUIRED)

# Install Python 노드
install(
  PROGRAMS
    planner/astar_path_planner.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS
    planner/goal_sender.py
  DESTINATION lib/${PROJECT_NAME}
)


# Install launch/config/map 디렉토리
install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
