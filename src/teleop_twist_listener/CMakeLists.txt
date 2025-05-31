cmake_minimum_required(VERSION 3.8)
project(teleop_twist_listener)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(teleop_listener src/teleop_listener.c)
ament_target_dependencies(teleop_listener rcl rclc geometry_msgs)

install(TARGETS teleop_listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
