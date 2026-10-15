import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('ugv_base')
    default_config = os.path.join(pkg_dir, 'config', 'ugv_base_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to ugv_base parameter YAML file'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Serial port of C50C control board'
        ),
        DeclareLaunchArgument(
            'heading',
            default_value='drive_front',
            description='Heading mode: drive_front or omni_front (ADR-0001)'
        ),

        Node(
            package='ugv_base',
            executable='ugv_base_node',
            name='ugv_base_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {
                    'port': LaunchConfiguration('port'),
                    'heading': LaunchConfiguration('heading'),
                }
            ],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
                ('/odom', '/odom'),
                ('/imu/data_raw', '/imu/data_raw'),
                ('/battery_state', '/battery_state')
            ]
        )
    ])
