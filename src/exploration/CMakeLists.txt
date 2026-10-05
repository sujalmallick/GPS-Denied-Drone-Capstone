cmake_minimum_required(VERSION 3.8)
project(exploration)

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
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Frontier detector node (M5-M6-M7 integrated)
add_executable(frontier_detector_node
  src/frontier_detector_node.cpp
  src/frontier_detector.cpp
  src/frontier_clustering.cpp
  src/frontier_selector.cpp
)
ament_target_dependencies(frontier_detector_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  std_msgs
)
target_include_directories(frontier_detector_node PUBLIC include)

# Install
install(TARGETS frontier_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
