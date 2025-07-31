#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition


def generate_launch_description():

    # --------------------------------------
    # >>> Declare launch arguments >>>
    # --------------------------------------

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='Simulation mode (sim or deploy)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    launch_viz_arg = DeclareLaunchArgument(
        'launch_viz',
        default_value='false',
        description='Launch RTAB-Map Viz visualization if true'
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='warn',
        description='Set the log level for RTAB-Map nodes'
    )

    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_viz = LaunchConfiguration('launch_viz')
    log_level = LaunchConfiguration('log_level')

    # --------------------------------------
    # >>> Launch description components >>>
    # --------------------------------------

    # Path to the RTAB-Map configuration file
    config_file_path = PathJoinSubstitution([
        FindPackageShare('flyscan_bringup'),
        'config',
        'rtabmap_config.yaml'
    ])

    # RTAB-Map node for 3D SLAM
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[config_file_path],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image'),
            ('scan_cloud', '/camera/depth/points'),
            ('odom', '/odom'),
            ('cloud_map', '/rtabmap/cloud_map'),
            ('imu', '/imu/data'),
            ('grid_map', '/rtabmap/grid_map'),
        ],
        output='screen',
        arguments=['--delete_db_on_start', '--ros-args', '--log-level', log_level],
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        parameters=[config_file_path],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image'),
            ('scan_cloud', '/camera/depth/points'),
            ('odom', '/odom'),
            ('cloud_map', '/rtabmap/cloud_map'),
            ('imu', '/imu/data'),
            ('grid_map', '/rtabmap/grid_map'),
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim' and '", launch_viz, "' == 'true'"])),
    )

    return LaunchDescription([
        use_sim_time_arg,
        launch_viz_arg,
        log_level_arg,
        mode_arg,
        rtabmap_node,
        rtabmap_viz,
    ])


    