from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import os

QGC_PATH = '~/Applications/QGroundControl-x86_64.AppImage'
PX4_DIR = '/home/ttd/Projects/ros2/px4_ws/PX4-Autopilot'
GZ_WORLD = 'warehouse_outdoor'

def generate_launch_description():
    sim_pkg = get_package_share_directory('flyscan_sim')

    gz_world = os.path.join(sim_pkg, 'worlds', f'{GZ_WORLD}.sdf')
    target_world = os.path.join(PX4_DIR, 'Tools', 'simulation', 'gz', 'worlds', f'{GZ_WORLD}.sdf')
    
    symlink_world = ExecuteProcess(
        cmd=['bash', '-c', f'ln -sf {gz_world} {target_world}'],
        output='screen'
    )

    px4_env_cmd = f'cd {PX4_DIR} && PX4_GZ_WORLD={GZ_WORLD} make px4_sitl_default gz_x500_depth'
    px4_sitl = ExecuteProcess(
        cmd=['bash', '-c', px4_env_cmd],
        output='screen'
    )

    qgc_cmd = f'pgrep -x QGroundControl || {QGC_PATH}'
    qgc = ExecuteProcess(
        cmd=['bash', '-c', qgc_cmd],
        output='screen'
    )

    micro_XRCE_bridge_cmd = 'MicroXRCEAgent udp4 -p 8888'
    micro_XRCE_bridge = ExecuteProcess(
        cmd=['bash', '-c', micro_XRCE_bridge_cmd],
        output='screen'
    )
    
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
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            
            '/world/warehouse_outdoor/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',

            # '/world/warehouse/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        remappings=[
            ('/depth_camera/camera_info', '/camera/camera_info'),
            ('/depth_camera/image', '/camera/image_raw'),
            ('/depth_camera/depth_image', '/camera/depth/image'),
            ('/depth_camera/points', '/camera/depth/points'),
            ('/clock', '/clock'),
            ('/world/warehouse_outdoor/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu', '/imu/data'),
        ],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        symlink_world,
        micro_XRCE_bridge,
        # qgc,
        px4_sitl,
        TimerAction(period=5.0, actions=(gz_bridge_node,)),
    ])
