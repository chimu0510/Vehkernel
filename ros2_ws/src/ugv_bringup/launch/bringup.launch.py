import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    desc_share = get_package_share_directory('ugv_description')
    nav_share = get_package_share_directory('ugv_navigation')
    bringup_share = get_package_share_directory('ugv_bringup')

    heading = LaunchConfiguration('heading').perform(context)
    serial_port = LaunchConfiguration('serial_port').perform(context)
    baud = int(LaunchConfiguration('baud').perform(context))
    use_ekf_str = LaunchConfiguration('use_ekf').perform(context).lower()
    use_ekf = use_ekf_str in ('true', '1', 'yes')

    use_teleop = LaunchConfiguration('use_teleop')
    use_lidar = LaunchConfiguration('use_lidar')
    run_slam = LaunchConfiguration('slam')
    run_nav2 = LaunchConfiguration('nav2')
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('rviz')

    actions = []

    # 1. Robot State Publisher (URDF xacro with heading)
    desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'heading': heading}.items(),
    )
    actions.append(desc_launch)

    # 2. Base Driver (ugv_base)
    # If EKF is enabled, ugv_base must not publish odom -> base_footprint TF
    base_node = Node(
        package='ugv_base',
        executable='ugv_base_node',
        name='ugv_base_node',
        parameters=[{
            'port': serial_port,
            'baud': baud,
            'heading': heading,
            'publish_tf': not use_ekf,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
        }],
        output='screen',
    )
    actions.append(base_node)

    # 3. EKF Filter (robot_localization)
    if use_ekf:
        ekf_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'ekf.launch.py')
            ),
        )
        actions.append(ekf_launch)

    # 4. Teleoperation (optional)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'teleop.launch.py')
        ),
        condition=IfCondition(use_teleop),
    )
    actions.append(teleop_launch)

    # 5. LiDAR Driver (optional)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'lidar.launch.py')
        ),
        condition=IfCondition(use_lidar),
    )
    actions.append(lidar_launch)

    # 6. SLAM Mapping (optional)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'slam.launch.py')
        ),
        condition=IfCondition(run_slam),
        launch_arguments={'rviz': use_rviz}.items(),
    )
    actions.append(slam_launch)

    # 7. Nav2 Navigation Stack (optional)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'navigation.launch.py')
        ),
        condition=IfCondition(run_nav2),
        launch_arguments={
            'heading': heading,
            'map': map_yaml,
            'rviz': use_rviz,
        }.items(),
    )
    actions.append(nav2_launch)

    return actions


def generate_launch_description():
    declare_heading = DeclareLaunchArgument(
        'heading',
        default_value='drive_front',
        choices=['drive_front', 'omni_front'],
        description='Software-defined heading mode (ADR-0001)',
    )

    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='C50C CDC/VCP USB serial port',
    )

    declare_baud = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='C50C serial baudrate',
    )

    declare_use_ekf = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='Enable robot_localization EKF sensor fusion (fusing odom + imu)',
    )

    declare_use_teleop = DeclareLaunchArgument(
        'use_teleop',
        default_value='false',
        description='Enable joystick teleoperation node',
    )

    declare_use_lidar = DeclareLaunchArgument(
        'use_lidar',
        default_value='false',
        description='Enable 2D LiDAR driver node',
    )

    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Launch SLAM Toolbox online async mapping',
    )

    declare_nav2 = DeclareLaunchArgument(
        'nav2',
        default_value='false',
        description='Launch Nav2 autonomous navigation stack',
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map yaml file for Nav2 localization',
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 for visualization',
    )

    return LaunchDescription([
        declare_heading,
        declare_serial_port,
        declare_baud,
        declare_use_ekf,
        declare_use_teleop,
        declare_use_lidar,
        declare_slam,
        declare_nav2,
        declare_map,
        declare_rviz,
        OpaqueFunction(function=launch_setup),
    ])
