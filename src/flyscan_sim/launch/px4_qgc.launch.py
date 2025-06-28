from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
import os

WORKSPACE_DIR = '/home/ttd/Documents/flyscan_ws'
QGC_PATH = '/home/ttd/Downloads/QGroundControl.AppImage'
PX4_DIR = '/home/ttd/Documents/flyscan_ws/PX4-Autopilot'

def generate_launch_description():
    px4_env_cmd = (
        f'cd {PX4_DIR} && '
        'PX4_GZ_WORLD=warehouse make px4_sitl_default gz_x500_depth'
    )

    micro_ros_agent_cmd = (
        'micro-xrce-dds-agent udp4 -p 8888'
    )
    
    qgc_conditional_cmd = (
        f'pgrep -x QGroundControl || {QGC_PATH}'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=[QGC_PATH],
            cwd=WORKSPACE_DIR,
            output='screen'
        ),
        ExecuteProcess(
            cmd=['bash', '-c', px4_env_cmd],
            cwd=PX4_DIR,
            output='screen'
        ),
        ExecuteProcess(
            cmd=['bash', '-c', micro_ros_agent_cmd],
            output='screen'
        )
    ])
