cmake_minimum_required(VERSION 3.8)
project(robot_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Define the executable before linking dependencies
add_executable(teleop_motor src/subscriber_teleop_key.cpp)

# Link dependencies correctly
ament_target_dependencies(teleop_motor rclcpp geometry_msgs)

install(TARGETS
  teleop_motor
  DESTINATION lib/${PROJECT_NAME})

ament_package()
