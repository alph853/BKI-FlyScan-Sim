from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():

    # --------------------------------------
    # >>> Declare launch arguments >>>
    # --------------------------------------

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='Launch mode (sim or deploy)'
    )
    rtabmap_launch_arg = DeclareLaunchArgument(
        'rtabmap_launch',
        default_value='false',
        description='Launch RTAB-Map 3D SLAM if true'
    )
    life_monitor_arg = DeclareLaunchArgument(
        'life_monitor',
        default_value='false',
        description='Launch life monitor node if true'
    )
    px4_controller_arg = DeclareLaunchArgument(
        'px4_controller',
        default_value='false',
        description='Launch PX4 controller node if true'
    )
    semantic_perception_arg = DeclareLaunchArgument(
        'semantic_perception',
        default_value='false',
        description='Launch semantic perception node if true'
    )

    mode = LaunchConfiguration('mode')
    use_sim_time    = LaunchConfiguration('use_sim_time')
    rtabmap_launch  = LaunchConfiguration('rtabmap_launch')
    life_monitor    = LaunchConfiguration('life_monitor')
    px4_controller  = LaunchConfiguration('px4_controller')
    semantic_perception = LaunchConfiguration('semantic_perception')

    # --------------------------------------
    # >>> Launch description components >>>
    # --------------------------------------

    # PX4 ROS Bridge node
    px4_ros_bridge_node = Node(
        package='flyscan_bridges',
        executable='px4_ros_bridge',
        name='px4_ros_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    # Gazebo-ROS2 bridge for simulation topics (sim mode only)
    gz_bridge_config_file = PathJoinSubstitution([
        FindPackageShare('flyscan_bringup'),
        'config',
        'gz_bridge_config.yaml'
    ])
    camera_topic_prefix = '/world/warehouse_outdoor/model/x500_depth_0/link/camera_link/sensor/StereoOV7251'
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[
            {'use_sim_time': use_sim_time},
            gz_bridge_config_file
        ],
        arguments=[
            f'{camera_topic_prefix}/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            f'{camera_topic_prefix}/image@sensor_msgs/msg/Image@gz.msgs.Image',
            f'{camera_topic_prefix}/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            f'{camera_topic_prefix}/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',

            '/world/warehouse_outdoor/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',

            '/world/warehouse_outdoor/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        remappings=[
            (f'{camera_topic_prefix}/camera_info', '/camera/camera_info'),
            (f'{camera_topic_prefix}/image', '/camera/image_raw'),
            (f'{camera_topic_prefix}/depth_image', '/camera/depth/image'),
            (f'{camera_topic_prefix}/points', '/camera/depth/points'),
            ('/world/warehouse_outdoor/clock', '/clock'),
            ('/world/warehouse_outdoor/model/x500_depth_0/link/base_link/sensor/imu_sensor/imu', '/imu/data'),
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))    
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
    
    # Static transform publisher for IMU to base_link
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

    # Visualization node (sim mode only)
    visualization_node = Node(
        package='flyscan_sim',
        executable='visualization_node',
        name='visualization_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_path_length': 1000,
        }],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))    
    )

    # Video streamer node (sim mode only)
    video_streamer_node = Node(
        package='flyscan_sim',
        executable='video_streamer',
        name='video_streamer',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))    
    )

    # RViz2 (sim mode only)
    rviz2 = ExecuteProcess(
        cmd=['rviz2'],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))    
    )

    rtabmap_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('flyscan_bringup'),
                'launch',
                'rtabmap_3d.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'mode': mode
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim' and '", rtabmap_launch, "' == 'true'"]))    
    )

    # LifeMonitor node
    life_monitor_node = Node(
        package='flyscan_core',
        executable='life_monitor',
        name='life_monitor',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        condition=IfCondition(life_monitor)
    )

    # PX4Controller nodegz_bridge_node
    px4_controller_node = Node(
        package='flyscan_drone_controller',
        executable='px4_controller',
        name='px4_controller',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        condition=IfCondition(px4_controller)
    )

    semantic_perception_node = Node(
        package='flyscan_perception',
        executable='semantic_perception',
        name='semantic_perception',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
        condition=IfCondition(semantic_perception)
    )

    return LaunchDescription([
        use_sim_time_arg,
        mode_arg,
        rtabmap_launch_arg,
        life_monitor_arg,
        px4_controller_arg,
        semantic_perception_arg,
        px4_ros_bridge_node,
        gz_bridge_node,
        static_tf_camera_to_base,
        static_tf_imu_to_base,
        visualization_node,
        video_streamer_node,
        rviz2,
        life_monitor_node,
        TimerAction(period=3.0, actions=(
            px4_controller_node,
            semantic_perception_node,
        )),
        rtabmap_launcher,
    ])