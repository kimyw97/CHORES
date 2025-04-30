cmake_minimum_required(VERSION 3.8)
project(stereo_litter_direct_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(stereo_litter_node src/stereo_litter_direct_node.cpp)
ament_target_dependencies(stereo_litter_node rclcpp std_msgs OpenCV)
ament_target_dependencies(stereo_litter_node ament_index_cpp)
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

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)
install(TARGETS stereo_litter_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
