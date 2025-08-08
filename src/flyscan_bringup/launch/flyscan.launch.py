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
    full_launch_enabled_arg = DeclareLaunchArgument(
        'full_launch_enabled',
        default_value='false',
        description='Launch full utility stack if true'
    )

    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    full_launch_enabled = LaunchConfiguration('full_launch_enabled')

    # --------------------------------------
    # >>> Launch description components >>>
    # --------------------------------------

    # Include PX4 simulation launch
    px4_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('flyscan_simulation'),
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
            "mode": mode,
            "use_sim_time": use_sim_time,
            "rtabmap_launch": PythonExpression(["'true' if '", full_launch_enabled, "' == 'true' else 'false'"]),
            "life_monitor": PythonExpression(["'true' if '", full_launch_enabled, "' == 'true' else 'false'"]),
            "px4_controller": PythonExpression(["'true' if '", full_launch_enabled, "' == 'true' else 'false'"]),
            # "semantic_perception": PythonExpression(["'true' if '", full_launch_enabled, "' == 'true' else 'false'"]),
            # "frontier_exploration": PythonExpression(["'true' if '", full_launch_enabled, "' == 'true' else 'false'"]),
        }.items()
    )

    return LaunchDescription([
        mode_arg,
        use_sim_time_arg,
        px4_sim_launch,
        full_launch_enabled_arg,
        TimerAction(
            period=5.0,
            actions=[utils_launch]
        ),
    ])