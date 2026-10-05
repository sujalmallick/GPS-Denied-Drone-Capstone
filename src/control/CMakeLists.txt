cmake_minimum_required(VERSION 3.8)
project(control)

# Default to C++17 (standard for ROS 2 Humble)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include)

# M2: Minimal PX4 control node
# This node demonstrates basic ROS 2 <-> PX4 communication
add_executable(px4_control_node src/px4_control_node.cpp)

ament_target_dependencies(
  px4_control_node
  rclcpp
  px4_msgs
  std_msgs
  std_srvs
  sensor_msgs
)

# Install targets
install(TARGETS
  px4_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
