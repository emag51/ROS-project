cd ros2_ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 robot_control

cd ros2_ws
colcon build --packages-select robot_control --symlink-install
