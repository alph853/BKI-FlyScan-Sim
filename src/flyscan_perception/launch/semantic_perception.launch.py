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

    # Semantic Perception node
    semantic_perception_node = Node(
        package='flyscan_perception',
        executable='semantic_perception',
        name='semantic_perception',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('flyscan_perception'),
                'config',
                'semantic_perception.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        semantic_perception_node,
    ])