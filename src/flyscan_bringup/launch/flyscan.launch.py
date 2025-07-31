from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    # --------------------------------------
    # >>> Declare launch arguments >>>
    # --------------------------------------

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='Simulation mode (sim or deploy)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    semantic_perception_arg = DeclareLaunchArgument(
        'semantic_perception',
        default_value='false',
        description='Launch semantic perception node if true'
    )

    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    semantic_perception = LaunchConfiguration('semantic_perception')

    # --------------------------------------
    # >>> Launch description components >>>
    # --------------------------------------

    # Include PX4 simulation launch
    px4_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('flyscan_sim'),
                'launch',
                'px4_sim.launch.py'
            ])
        ]),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'sim'"]))    
    )

    # Include utilities launch
    utils_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('flyscan_bringup'),
                'launch',
                'utils.launch.py'
            ])
        ]),
        launch_arguments={
            'mode': mode
        }.items()
    )

    # Include RTAB-Map launch (sim mode only)
    rtabmap_launch = IncludeLaunchDescription(
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
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"]))    
    )

    # LifeMonitor node
    life_monitor_node = Node(
        package='flyscan_core',
        executable='life_monitor',
        name='life_monitor',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # PX4Controller nodegz_bridge_node
    px4_controller_node = Node(
        package='flyscan_drone_controller',
        executable='px4_controller',
        name='px4_controller',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
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
        mode_arg,
        use_sim_time_arg,
        px4_sim_launch,
        utils_launch,
        semantic_perception_arg,
        life_monitor_node,
        px4_controller_node,
        semantic_perception_node,
        rtabmap_launch,
    ])