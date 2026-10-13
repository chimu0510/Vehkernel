import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_navigation')
    default_ekf_params = os.path.join(pkg_share, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_params_file = LaunchConfiguration('ekf_params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_ekf_params = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=default_ekf_params,
        description='Path to EKF parameter file',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odometry/filtered')],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_ekf_params,
        ekf_node,
    ])
