#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Path to the RTAB-Map configuration file
    config_file_path = PathJoinSubstitution([
        FindPackageShare('flyscan_sim'),
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
        arguments=['--delete_db_on_start']
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
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        rtabmap_node,
        # rtabmap_viz,
    ])


    