#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    delete_db_arg = DeclareLaunchArgument(
        'delete_db',
        default_value='true',
        description='Delete RTAB-Map database on start'
    )
    
    enable_viz_arg = DeclareLaunchArgument(
        'enable_viz',
        default_value='false',
        description='Enable RTAB-Map visualization'
    )

    rtabmap_config = PathJoinSubstitution([
        FindPackageShare('flyscan_sim'),
        'config',
        'rtabmap_config.yaml'
    ])

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[rtabmap_config],
        arguments=[
            '--delete_db_on_start',
            '--Mem/IncrementalMemory', 'true',
            '--Mem/InitWMWithAllNodes', 'false'
        ],
        remappings=[
            ('imu', '/imu/data'),
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image'),
            ('scan_cloud', '/camera/depth/points'),
            ('odom', '/fmu/out/vehicle_odometry'),
        ],
        output='screen',
        condition=IfCondition('true')
    )

    rtabmapviz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmapviz',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=IfCondition(LaunchConfiguration('enable_viz')),
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        delete_db_arg,
        enable_viz_arg,
        
        rtabmap_node,
        rtabmapviz_node,
    ])
