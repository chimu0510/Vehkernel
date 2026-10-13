import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_bringup')
    default_config = os.path.join(pkg_share, 'config', 'teleop_joy.yaml')

    joy_dev = LaunchConfiguration('joy_dev')
    config_file = LaunchConfiguration('config_file')

    declare_joy_dev = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick Linux device path',
    )

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to teleop joy parameter file',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'device_name': joy_dev,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen',
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_file],
        remappings=[('/cmd_vel', '/cmd_vel')],
        output='screen',
    )

    return LaunchDescription([
        declare_joy_dev,
        declare_config_file,
        joy_node,
        teleop_node,
    ])
