import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('ugv_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'ugv.urdf.xacro')

    heading_arg = DeclareLaunchArgument(
        'heading',
        default_value='drive_front',
        description='Heading mode: drive_front or omni_front (ADR-0001)'
    )

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        xacro_file, ' ',
        'heading:=', LaunchConfiguration('heading')
    ])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        heading_arg,
        rsp_node,
        jsp_gui_node,
        rviz_node
    ])
