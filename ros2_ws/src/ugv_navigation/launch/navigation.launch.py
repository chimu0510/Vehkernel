import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('ugv_navigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    heading = LaunchConfiguration('heading').perform(context)
    params_file_input = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml_file = LaunchConfiguration('map').perform(context)
    use_rviz = LaunchConfiguration('rviz')

    if not params_file_input:
        if heading == 'omni_front':
            params_file = os.path.join(pkg_share, 'config', 'nav2_params_omni_front.yaml')
        else:
            params_file = os.path.join(pkg_share, 'config', 'nav2_params_drive_front.yaml')
    else:
        params_file = params_file_input

    rviz_config_file = os.path.join(pkg_share, 'rviz', 'nav2.rviz')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return [nav2_bringup, rviz_node]


def generate_launch_description():
    declare_heading = DeclareLaunchArgument(
        'heading',
        default_value='drive_front',
        choices=['drive_front', 'omni_front'],
        description='Robot heading configuration (ADR-0001)',
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Full path to Nav2 parameter file (defaults based on heading if empty)',
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load for Nav2 localization',
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start the Nav2 lifecycle stack',
    )

    declare_use_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 for Nav2 visualization',
    )

    return LaunchDescription([
        declare_heading,
        declare_use_sim_time,
        declare_params_file,
        declare_map,
        declare_autostart,
        declare_use_rviz,
        OpaqueFunction(function=launch_setup),
    ])
