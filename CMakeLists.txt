cmake_minimum_required(VERSION 3.5)
project(robot_pose_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  include/${PROJECT_NAME}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(robot_pose_publisher_node src/robot_pose_publisher.cpp)
ament_target_dependencies(robot_pose_publisher_node geometry_msgs rclcpp tf2_ros)

install(TARGETS
  robot_pose_publisher_node
  DESTINATION lib/${PROJECT_NAME})
ament_package()
