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

colcon build --packages-select robot_control

Running
cd ~/ros2_ws
. install/setup.bash
ros2 run robot_control listener

another terminal run teleop_twist_key

ros2 run teleop_twist_keyboard teleop_twist_keyboard
