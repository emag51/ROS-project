import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Get package path
    package_name = 'robot_urdf'
    package_path = get_package_share_directory(package_name)

    # Ensure model path is absolute
    default_model_path = os.path.join(package_path, 'urdf', '05-visual.urdf')

    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        'model', 
        default_value=default_model_path,  # FIX: Ensure the URDF path is absolute
        description='Absolute path to robot URDF file'
    )

    gui_arg = DeclareLaunchArgument(
        'gui', 
        default_value='true', 
        description='Launch joint state publisher GUI'
    )

    rviz_arg = DeclareLaunchArgument(
        'rvizconfig', 
        default_value=os.path.join(package_path, 'rviz', 'urdf.rviz'),
        description='Path to RViz config file'
    )

    # Nodes
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_no_gui_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('gui'), "' == 'false'"]))
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
    )

    # Launch description
    return LaunchDescription([
        model_arg,
        gui_arg,
        rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_no_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
