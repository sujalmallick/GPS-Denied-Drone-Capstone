cmake_minimum_required(VERSION 3.8)
project(localization)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

# Include directories
include_directories(include)

# Ground-truth pose publisher node
add_executable(ground_truth_pose_publisher
  src/ground_truth_pose_publisher.cpp
)
ament_target_dependencies(ground_truth_pose_publisher
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  gazebo_ros
)

# Camera/Depth bridge node
add_executable(sensor_bridge
  src/sensor_bridge.cpp
)
ament_target_dependencies(sensor_bridge
  rclcpp
  geometry_msgs
  sensor_msgs
)

# Install
install(TARGETS
  ground_truth_pose_publisher
  sensor_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
