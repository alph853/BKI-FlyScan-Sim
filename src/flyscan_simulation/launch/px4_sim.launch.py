from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

import os
from dataclasses import dataclass


PX4_DIR  = '/home/ttd/Projects/ros2/px4_ws/PX4-Autopilot'
GZ_WORLD = 'warehouse_outdoor'


@dataclass
class DroneConfig:
    pose: str
    id: int | None = None


SWARM = [
    DroneConfig(id=0,pose="0,0"),
    DroneConfig(id=1,pose="0,5"),
    DroneConfig(id=2,pose="0,10"),
]


def generate_launch_description():

    # --------------------------------------
    # >>> Declare launch arguments >>>
    # --------------------------------------

    swarm_enabled_arg = DeclareLaunchArgument(
        'swarm_enabled',
        default_value='false',
        description='Launch PX4 SITL swarm simulation if true'
    )
    symlink_world_arg = DeclareLaunchArgument(
        'symlink_world',
        default_value='false',
        description='Symlink world files to PX4 simulation directory if true'
    )

    swarm_enabled = LaunchConfiguration('swarm_enabled')
    symlink_world = LaunchConfiguration('symlink_world')

    # --------------------------------------
    # >>> Launch description components >>>
    # --------------------------------------

    sim_pkg     = get_package_share_directory('flyscan_simulation')

    # Symlink world files to PX4 simulation directory
    worlds_path  = os.path.join(sim_pkg, 'worlds')
    target_worlds_path = os.path.join(PX4_DIR, 'Tools', 'simulation', 'gz', 'worlds')
    symlink_world = ExecuteProcess(
        cmd=['bash', '-c', f'ln -sf {worlds_path} {target_worlds_path}'],
        output='screen',
        condition=IfCondition(symlink_world)
    )

    # PX4 SITL command
    px4_cmd = f'cd {PX4_DIR} && make px4_sitl_default gz_x500_flyscan_{GZ_WORLD}'
    px4_sitl = ExecuteProcess(
        cmd=['bash', '-c', px4_cmd],
        output='screen',
    )

    # Micro XRCE-DDS bridge command
    micro_XRCE_bridge_cmd = 'MicroXRCEAgent udp4 -p 8888'
    micro_XRCE_bridge = ExecuteProcess(
        cmd=['bash', '-c', micro_XRCE_bridge_cmd],
        output='screen'
    )

    swarm_list = [
        ExecuteProcess(
            cmd=['bash', '-c', f'cd {PX4_DIR} && PX4_SYS_AUTOSTART=22999 PX4_GZ_MODEL_POSE="{drone.pose}" ./build/px4_sitl_default/bin/px4 -i {drone.id}'],
            output='screen',
        ) for drone in SWARM[1:]
    ]
    swarm_cmd = TimerAction(
        period=10.0,
        actions=swarm_list,
        condition=IfCondition(swarm_enabled)
    )

    return LaunchDescription([
        swarm_enabled_arg,
        symlink_world_arg,
        symlink_world,
        micro_XRCE_bridge,
        px4_sitl,
        swarm_cmd
    ])
