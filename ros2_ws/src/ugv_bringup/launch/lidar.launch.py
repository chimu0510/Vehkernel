import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_bringup')
    default_config = os.path.join(pkg_share, 'config', 'sllidar_params.yaml')

    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    frame_id = LaunchConfiguration('frame_id')

    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='LiDAR serial port device',
    )

    declare_serial_baud = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='LiDAR serial baudrate',
    )

    declare_frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_link',
        description='LiDAR TF frame ID',
    )

    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[default_config, {
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
        }],
        output='screen',
    )

    return LaunchDescription([
        declare_serial_port,
        declare_serial_baud,
        declare_frame_id,
        lidar_node,
    ])
