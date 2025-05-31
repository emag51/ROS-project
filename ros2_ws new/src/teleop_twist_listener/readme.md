# running this code
Example : https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

ROS structure
User/ros2_ws/src/robot_control

create a package (robot_control)

source /opt/ros/humble/setup.bash

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

ros2 pkg create --build-type ament_cmake --license Apache-2.0 robot_control

Adjust code
create subscriber_teleop_key.cpp in /ros2_ws/src/robot_control/src

change CMakeLists.txt and package.xml

Building package
cd ~/ros2_ws/

colcon build --packages-select teleop_twist_listener --symlink-install
colcon build --packages-select teleop_twist_listener --symlink-install
Running
cd ~/ros2_ws
source install/setup.bash
./install/teleop_twist_listener/lib/teleop_twist_listener/teleop_listener

another terminal run teleop_twist_key
cd ~/ros2_ws
ros2 run teleop_twist_keyboard teleop_twist_keyboard