from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


PX4_DIR  = '/home/ttd/Projects/ros2/px4_ws/PX4-Autopilot'
GZ_WORLD = 'warehouse_outdoor'


def generate_launch_description():

    sim_pkg     = get_package_share_directory('flyscan_simulation')

    worlds_path  = os.path.join(sim_pkg, 'worlds')
    target_worlds_path = os.path.join(PX4_DIR, 'Tools', 'simulation', 'gz', 'worlds')
    symlink_world = ExecuteProcess(
        cmd=['bash', '-c', f'ln -sf {worlds_path} {target_worlds_path}'],
        output='screen'
    )
    print(f'Symlinking worlds from {worlds_path} to {target_worlds_path}')
    px4_env_cmd = f'cd {PX4_DIR} && make px4_sitl_default gz_x500_depth'
    px4_sitl = ExecuteProcess(
        cmd=['bash', '-c', px4_env_cmd],
        output='screen',
        additional_env={
            'PX4_GZ_WORLD': GZ_WORLD,
        }
    )

    micro_XRCE_bridge_cmd = 'MicroXRCEAgent udp4 -p 8888'
    micro_XRCE_bridge = ExecuteProcess(
        cmd=['bash', '-c', micro_XRCE_bridge_cmd],
        output='screen'
    )

    return LaunchDescription([
        symlink_world,
        micro_XRCE_bridge,
        px4_sitl,
    ])
