#!/usr/bin/env python3

"""
Launch file for teleop_node.

This launch file starts the teleop_node which provides keyboard-based
teleoperation control for the px4_controller.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for teleop node."""
    
    teleop_node = Node(
        package='flyscan_drone_controller',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        parameters=[
            # Add any parameters here if needed
        ]
    )
    
    return LaunchDescription([
        teleop_node
    ])