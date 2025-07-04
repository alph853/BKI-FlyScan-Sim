#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Path to SLAM Toolbox configuration file
    slam_config_file = PathJoinSubstitution([
        FindPackageShare('slam_toolbox'),
        'config',
        'mapper_params_online_async.yaml'
    ])

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
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
    
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/camera/depth/points'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.2,      # More relaxed TF tolerance
            'min_height': -1.0,              # Much taller slice to capture all relevant points
            'max_height': 1.0,
            'angle_min': -3.14159,           # Full 360 deg if sensor allows it
            'angle_max': 3.14159,
            'angle_increment': 0.00872665,   # ~0.5 degrees for more detail (~720 points)
            'scan_time': 0.0,
            'range_min': 0.2,
            'range_max': 10.0,               # Longer range to include distant walls
            'use_inf': True,                 # Keep inf for missing data (standard for SLAM)
            'use_sim_time': use_sim_time,
            'concurrency_level': 1,          # Keep default, multi-threading rarely helps here
            'queue_size': 10,                # Smaller queue to avoid stale data
        }]
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_config_file,
            {
                'use_sim_time': use_sim_time,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'use_scan_matching': True,
                'use_scan_barycenter': True,
                'minimum_time_interval': 0.5,
                'transform_publish_period': 0.02,
                'map_update_interval': 5.0,
                'resolution': 0.05,
                'max_laser_range': 5.0,
                'minimum_travel_distance': 0.5,
                'minimum_travel_heading': 0.5,
                'laserMaxRange': 5.0,
                'useLaserScanBaryCentering': True,
                'minimumTimeInterval': 0.5,
                'transformPublishPeriod': 0.02,
                'mapUpdateInterval': 5.0,
                "mode": "mapping",
            }
        ],
        remappings=[
            ('scan', '/scan'),
            # ('odom', '/odom'),
        ],
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
            'use_sim_time': True
        }]
    )

    # static_tf_camera_to_base = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='camera_to_base_link',
    #     arguments=[
    #         '0',     # x
    #         '0',     # y  
    #         '0.3',   # z
    #         '0',     # roll
    #         '0',     # pitch
    #         '0',     # yaw
    #         'base_link',  # parent frame
    #         'x500_depth_0/camera_link/StereoOV7251'  # child frame
    #     ],
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #     }]
    # )

    configure_slam = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
        shell=False
    )
    
    # Activate the node
    activate_slam =  ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
        shell=False
    )

    return LaunchDescription([
        use_sim_time_arg,
        odom_bridge_node,
        static_tf_camera_to_base,
        slam_toolbox_node,
        TimerAction(period=1.0, actions=(configure_slam,)),
        TimerAction(period=2.0, actions=(activate_slam,)),
        TimerAction(period=5.0, actions=(pointcloud_to_laserscan_node,)),
    ])