#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    
    # Launch arguments
    exploration_radius_arg = DeclareLaunchArgument(
        'exploration_radius',
        default_value='10.0',
        description='Maximum exploration radius in meters'
    )
    
    min_frontier_size_arg = DeclareLaunchArgument(
        'min_frontier_size',
        default_value='5',
        description='Minimum frontier size to consider'
    )
    
    frontier_cluster_distance_arg = DeclareLaunchArgument(
        'frontier_cluster_distance',
        default_value='2.0',
        description='Distance for clustering frontier cells'
    )
    
    exploration_rate_arg = DeclareLaunchArgument(
        'exploration_rate',
        default_value='1.0',
        description='Exploration update rate in Hz'
    )
    
    start_exploration_arg = DeclareLaunchArgument(
        'start_exploration',
        default_value='false',
        description='Start exploration automatically'
    )
    
    # Configuration
    exploration_radius = LaunchConfiguration('exploration_radius')
    min_frontier_size = LaunchConfiguration('min_frontier_size')
    frontier_cluster_distance = LaunchConfiguration('frontier_cluster_distance')
    exploration_rate = LaunchConfiguration('exploration_rate')
    start_exploration = LaunchConfiguration('start_exploration')
    
    # Frontier Explorer Node
    frontier_explorer_node = Node(
        package='flyscan_exploration',
        executable='frontier_explorer_node',
        name='frontier_explorer',
        parameters=[{
            'exploration_radius': exploration_radius,
            'min_frontier_size': min_frontier_size,
            'frontier_cluster_distance': frontier_cluster_distance,
            'exploration_rate': exploration_rate,
        }],
        remappings=[
            ('/rtabmap/grid_map', '/map'),
        ],
        output='screen',
        condition=IfCondition(start_exploration)
    )
    
    return LaunchDescription([
        exploration_radius_arg,
        min_frontier_size_arg,
        frontier_cluster_distance_arg,
        exploration_rate_arg,
        start_exploration_arg,
        frontier_explorer_node,
    ])