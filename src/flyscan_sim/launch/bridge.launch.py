#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    bridge_config_file = PathJoinSubstitution([
        FindPackageShare('flyscan_sim'),
        'config',
        'bridge_config.yaml'
    ])

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            bridge_config_file
        ],
        arguments=[
            '/world/warehouse/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            
            '/world/warehouse/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/world/warehouse/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image',
            
            # '/world/warehouse/model/x500_depth_0/link/camera_link/sensor/camera_imu/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',

            # '/model/x500_depth_0/odometry_with_covariance@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance',
            
            '/world/warehouse/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            
            # '/world/warehouse/model/x500_depth_0/link/camera_link/sensor/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            # '/world/warehouse/model/x500_depth_0/link/camera_link/sensor/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/world/warehouse/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu', '/imu/data'),
            ('/world/warehouse/model/x500_depth_0/link/camera_link/sensor/IMX214/image', '/camera/image_raw'),
            ('/world/warehouse/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info', '/camera/camera_info'),
            ('/depth_camera', '/camera/depth/image'),
            ('/depth_camera/points', '/camera/depth/points'),
        ],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        
        gz_bridge_node,
    ])