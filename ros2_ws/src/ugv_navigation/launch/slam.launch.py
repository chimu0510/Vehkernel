import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_navigation')
    default_slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_rviz = LaunchConfiguration('rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )

    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params,
        description='Full path to SLAM Toolbox configuration yaml',
    )

    declare_use_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 for SLAM monitoring',
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        arguments=['-d', default_rviz_config],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params,
        declare_use_rviz,
        slam_node,
        rviz_node,
    ])
