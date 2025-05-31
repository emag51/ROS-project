cmake_minimum_required(VERSION 3.5)
project(robot_control)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(robot_control src/robot_control.c)
ament_target_dependencies(robot_control rcl rclc geometry_msgs)

install(TARGETS
  robot_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
