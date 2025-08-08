#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('flyscan_exploration'),
            'config',
            'sample_mission.yaml'
        ]),
        description='Path to the waypoint mission YAML file'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Automatically start the mission when node is activated'
    )
    
    mission_rate_arg = DeclareLaunchArgument(
        'mission_rate',
        default_value='5.0',
        description='Mission update rate in Hz'
    )
    
    # Waypoint mission node
    waypoint_mission_node = Node(
        package='flyscan_exploration',
        executable='waypoint_mission',
        name='waypoint_mission',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'default_mission_file': LaunchConfiguration('mission_file'),
            'auto_start_mission': LaunchConfiguration('auto_start'),
            'mission_update_rate': LaunchConfiguration('mission_rate'),
        }],
        # Use component manager for better performance (optional)
        # node_executable='component_container',
        # node_namespace='',
        # composable_node_descriptions=[
        #     ComposableNode(
        #         package='flyscan_exploration',
        #         plugin='flyscan::exploration::WaypointMission',
        #         name='waypoint_mission',
        #         parameters=[{...}]
        #     )
        # ]
    )
    
    return LaunchDescription([
        mission_file_arg,
        auto_start_arg,
        mission_rate_arg,
        waypoint_mission_node
    ])