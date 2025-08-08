from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # PX4Controller node
    px4_controller_node = Node(
        package='flyscan_drone_controller',
        executable='px4_controller',
        name='px4_controller',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('flyscan_drone_controller'),
                'config',
                'px4_controller.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        px4_controller_node,
    ])