from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():
    return LaunchDescription([
        # Include robot URDF publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("my_robot_description"),
                    "launch",
                    "robot_description.launch.py"
                ])
            ])
        ),

        # RViz with custom config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(Path(__file__).parent / '../rviz/slam_toolbox.rviz')],
            output='screen',
        ),

        # # RPLIDAR
        # Node(
        #     package='rplidar_ros',
        #     executable='rplidar_node',
        #     name='rplidar',
        #     output='screen',
        #     parameters=[
        #         {'serial_port': '/dev/ttyUSB0'},
        #         {'serial_baudrate': 115200},
        #         {'frame_id': 'laser'}
        #     ],
        # ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'base_frame': 'laser'},  # Change to 'base_link' if using URDF model
                {'odom_frame': 'odom'},
                {'scan_topic': '/scan'}
            ],
        ),

        # Static TF for laser to base_link (only if needed)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        ),
    ])
