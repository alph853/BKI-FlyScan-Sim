#!/usr/bin/env python3

"""
Launch file for PX4Controller in a composable container.

This launch file creates a component container and loads the PX4Controller
as a composable node within that container for improved performance.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for PX4Controller composable container."""

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch arguments
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='px4_controller',
        description='Name of the PX4Controller node'
    )
    
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='px4_controller_container',
        description='Name of the composable container'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the nodes'
    )

    use_intra_process_comms_arg = DeclareLaunchArgument(
        'use_intra_process_comms',
        default_value='true',
        description='Use intra-process communications for better performance'
    )
    
    use_multithreaded_executor_arg = DeclareLaunchArgument(
        'use_multithreaded_executor',
        default_value='true',
        description='Use multithreaded executor (component_container_mt) for better callback processing'
    )

    def launch_setup(context, *args, **kwargs):
        """Setup the launch based on context."""
        
        node_name = LaunchConfiguration('node_name')
        container_name = LaunchConfiguration('container_name')
        namespace = LaunchConfiguration('namespace')
        use_intra_process_comms = LaunchConfiguration('use_intra_process_comms')
        use_multithreaded_executor = LaunchConfiguration('use_multithreaded_executor')
        
        # Define the PX4Controller composable node
        px4_controller_node = ComposableNode(
            package='flyscan_drone_controller',
            plugin='flyscan::drone_controller::PX4Controller',
            name=node_name,
            namespace=namespace,
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('flyscan_drone_controller'),
                    'config',
                    'px4_controller.yaml'
                ]),
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                # Add any topic remappings here if needed    
            ],
            extra_arguments=[
                {'use_intra_process_comms': use_intra_process_comms}
            ]
        )
        

        life_monitor_node = ComposableNode(
            package='flyscan_core',
            plugin='flyscan::core::LifeMonitor',
            name='life_monitor',
            namespace=namespace,
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('flyscan_core'),
                    'config',
                    'base_node.yaml'
                ]),
                {'use_sim_time': use_sim_time}
            ]
        )

        use_mt = context.perform_substitution(use_multithreaded_executor).lower() == 'true'
        container_executable = 'component_container_mt' if use_mt else 'component_container'

        container = ComposableNodeContainer(
            name=container_name,
            namespace=namespace,
            package='rclcpp_components',
            executable=container_executable,
            composable_node_descriptions=[
                px4_controller_node,
                life_monitor_node
            ],
            output='screen',
            parameters=[
                {'use_intra_process_comms': use_intra_process_comms}
            ]
        )
        
        return [container]

    configure_px4_controller = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/px4_controller', 'configure'],
        output='screen'
    )
    
    activate_px4_controller = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/px4_controller', 'activate'],
        output='screen'
    )

    return LaunchDescription([
        node_name_arg,
        container_name_arg,
        namespace_arg,
        use_sim_time_arg,
        use_intra_process_comms_arg,
        use_multithreaded_executor_arg,
        OpaqueFunction(function=launch_setup),
        TimerAction(period=1.0, actions=[configure_px4_controller]),
        TimerAction(period=2.0, actions=[activate_px4_controller]),
    ])
