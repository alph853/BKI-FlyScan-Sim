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
    
    # Odometry bridge node from flyscan_bridges
    odom_bridge_node = Node(
        package='flyscan_bridges',
        executable='odom_bridge',
        name='odom_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
            'debug': True,
            'max_path_length': 1000,
        }],
        output='screen'
    )

    # Static transform publisher for camera to base_link
    static_tf_camera_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_link',
        arguments=[
            '--x', '0.13233',
            '--y', '0.0',
            '--z', '0.26078',
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
    # Static transform to offset map frame to align with PX4 coordinates
    static_tf_map_offset = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_offset_publisher',
        arguments=[
            '--x', '-2',
            '--y', '2', 
            '--z', '0',
            '--qx', '0',
            '--qy', '0',
            '--qz', '0',
            '--qw', '1',
            '--frame-id', 'map',
            '--child-frame-id', 'odom'
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Marker publisher for coordinate systems and room labels
    marker_publisher_node = Node(
        package='flyscan_sim',
        executable='marker_publisher',
        name='marker_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    rviz2 = ExecuteProcess(
        cmd=['rviz2'],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        odom_bridge_node,
        static_tf_camera_to_base,
        static_tf_imu_to_base,
        # static_tf_map_offset,
        marker_publisher_node,
        rviz2,
    ])