"""Preview every Wheeltec chassis model in RViz.

  ros2 launch ugv_model_viewer view_models.launch.py
  ros2 launch ugv_model_viewer view_models.launch.py model:=top_4wd_bs_robot
  ros2 launch ugv_model_viewer view_models.launch.py cycle_period:=5.0
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ugv_model_viewer.catalog import Catalog

SHARE = Path(get_package_share_directory('ugv_model_viewer'))


def _nodes(context):
    urdf_dir = LaunchConfiguration('urdf_dir').perform(context)
    model = LaunchConfiguration('model').perform(context)
    catalog = Catalog(urdf_dir)
    model = model or catalog.models[0]
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': catalog.description(model)}]),
        Node(package='ugv_model_viewer', executable='model_viewer', output='screen',
             parameters=[{'urdf_dir': urdf_dir, 'model': model,
                          'cycle_period': float(LaunchConfiguration('cycle_period')
                                                .perform(context))}]),
    ]


def generate_launch_description():
    default_urdf_dir = (SHARE / 'model_urdf_dir.txt').read_text(encoding='utf-8').strip()
    return LaunchDescription([
        DeclareLaunchArgument('urdf_dir', default_value=default_urdf_dir,
                              description='Directory holding the *.urdf files'),
        DeclareLaunchArgument('model', default_value='',
                              description='Model to show first (URDF file stem)'),
        DeclareLaunchArgument('cycle_period', default_value='0.0',
                              description='Seconds per model when auto-cycling; 0 disables'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_nodes),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(SHARE / 'rviz' / 'model_viewer.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
