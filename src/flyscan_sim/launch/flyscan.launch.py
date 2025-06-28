#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


WORKSPACE_DIR = '/home/ttd/Documents/flyscan_ws'
QGC_PATH = '/home/ttd/Downloads/QGroundControl.AppImage'
PX4_DIR = '/home/ttd/Documents/flyscan_ws/PX4-Autopilot'

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('flyscan_sim'),
            'rviz',
            'flyscan_rtabmap.rviz'
        ]),
        description='Path to RViz config file'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    QGroundControl = ExecuteProcess(
        cmd=[QGC_PATH],
        cwd=WORKSPACE_DIR,
        output='screen'
    )

    px4_gz_sim = ExecuteProcess(
        cmd=['bash', '-c', 'PX4_GZ_WORLD=warehouse make px4_sitl_default gz_x500_depth'],
        cwd=PX4_DIR,
        output='screen'
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '/world/warehouse/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            
            '/world/warehouse/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/world/warehouse/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image',
            
            '/world/warehouse/model/x500_depth_0/link/camera_link/sensor/camera_imu/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',

            '/model/x500_depth_0/odometry_with_covariance@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance',
            
            '/world/warehouse/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        remappings=[
            ('/world/warehouse/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu', '/imu/data'),
            ('/world/warehouse/model/x500_depth_0/link/camera_link/sensor/IMX214/image', '/camera/image_raw'),
            ('/world/warehouse/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info', '/camera/camera_info'),
            ('/world/warehouse/model/x500_depth_0/link/camera_link/sensor/camera_imu/imu', '/camera/imu'),
            ('/depth_camera/points', '/camera/depth/points'),
            ('/model/x500_depth_0/odometry_with_covariance', '/odom'),
        ],
        output='screen',
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': False,
                'subscribe_scan_cloud': True,
                'subscribe_stereo': False,
                'subscribe_imu': True,

                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'queue_size': 10,
                'approx_sync': True,
                
                'Rtabmap/DetectionRate': '1.0',
                'Rtabmap/TimeThr': '0',
                'Mem/RehearsalSimilarity': '0.30',
                'Mem/NotLinkedNodesKept': 'false',
                'RGBD/OptimizeFromGraphEnd': 'false',
                'RGBD/ProximityPathMaxNeighbors': '0',
                'RGBD/NeighborLinkRefining': 'true',
                'Grid/FromDepth': 'true',
                'Grid/CellSize': '0.05',
                'Grid/DepthMax': '4.0',
                'Grid/MaxObstacleHeight': '2.0',
                'Grid/MaxGroundHeight': '0.1',
                'Grid/ClusterRadius': '0.1',
                'Grid/MinClusterSize': '10',
                'Grid/FlatObstacleDetected': 'true',
                
                'Vis/FeatureType': '6',
                'Vis/MaxFeatures': '1000',
                'Vis/MinInliers': '15',
                'Vis/EstimationType': '1',
                
                'Mem/STMSize': '30',
                'Mem/LaserScanDownsampleStepSize': '1',
            }
        ],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('scan_cloud', '/camera/depth/points'),
            ('odom', '/odom'),
            ('imu', '/imu/data'),
        ],
        arguments=[
            '--delete_db_on_start'
        ],
        output='screen',
    )

    rtabmapviz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmapviz',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )
    
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link',
        arguments=[
            '0.1', '0', '0.05',  # x, y, z
            '0', '0', '0', '1',  # qx, qy, qz, qw
            'base_link', 'camera_link'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': '<?xml version="1.0"?><robot name="x500_depth"><link name="base_link"/><link name="camera_link"/><joint name="camera_joint" type="fixed"><parent link="base_link"/><child link="camera_link"/><origin xyz="0.1 0 0.05" rpy="0 0 0"/></joint></robot>'}
        ],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_config_arg,
        enable_rviz_arg,
        
        QGroundControl,
        px4_gz_sim,
        TimerAction(period=10.0, actions=[gz_bridge_node]),
        TimerAction(period=12.0, actions=[rtabmap_node]),
        TimerAction(period=15.0, actions=[rtabmapviz_node]),
        TimerAction(period=17.0, actions=[base_to_camera_tf]),
        TimerAction(period=20.0, actions=[robot_state_publisher]),
        TimerAction(period=25.0, actions=[rviz_node]),
    ])