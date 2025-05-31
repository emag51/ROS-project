from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path

def generate_launch_description():
    return LaunchDescription([
        # Robot description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("my_robot_description"),
                    "launch",
                    "robot_description.launch.py"
                ])
            ])
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(Path(__file__).parent / '../rviz/slam_toolbox.rviz')],
            output='screen',
        ),

        # # SLAM Toolbox
        # Node(
        #     package='slam_toolbox',
        #     executable='sync_slam_toolbox_node',
        #     name='slam_toolbox',
        #     output='screen',
        #     parameters=[
        #         {'use_sim_time': False},
        #         {'base_frame': 'base_link'},
        #         {'odom_frame': 'odom'},
        #         {'scan_topic': '/scan'}
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
                {'slam_mode': True}, 
                {'base_frame': 'base_link'},
                {'odom_frame': 'odom'},
                {'scan_topic': '/scan'}
            ],
        ),


        # Static transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        ),
    ])
