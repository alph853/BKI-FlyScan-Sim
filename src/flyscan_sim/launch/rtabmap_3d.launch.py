#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

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
    
    # Odometry bridge node from flyscan_vision
    odom_bridge_node = Node(
        package='flyscan_vision',
        executable='odom_bridge',
        name='odom_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    # Static transform publisher for camera to base_link
    static_tf_camera_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_link',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0.3',
            '--qx', '0',
            '--qy', '0',
            '--qz', '0',
            '--qw', '1',
            '--frame-id', 'base_link',
            '--child-frame-id', 'x500_depth_0/camera_link/StereoOV7251'
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Static transform publisher for camera to base_link
    static_tf_imu_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_base_link',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--qx', '0',
            '--qy', '0',
            '--qz', '0',
            '--qw', '1',
            '--frame-id', 'base_link',
            '--child-frame-id', 'x500_depth_0/base_link/imu_sensor'
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

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
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        output='screen'
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
        odom_bridge_node,
        static_tf_camera_to_base,
        static_tf_imu_to_base,
        rtabmap_node,
        # rviz_node,
        rtabmap_viz,
    ])