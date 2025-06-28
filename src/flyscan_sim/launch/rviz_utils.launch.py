#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
        'enable_rviz',
        default_value='true',
        description='Enable RViz2'
    )
    
    enable_robot_state_arg = DeclareLaunchArgument(
        'enable_robot_state',
        default_value='true',
        description='Enable robot state publisher'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
        output='screen',
    )

    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link_tf',
        arguments=[
            '0.1', '0', '0.05',  # x, y, z
            '0', '0', '0', '1',  # qx, qy, qz, qw (quaternion)
            'base_link', 'camera_link'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    camera_to_depth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_depth_tf',
        arguments=[
            '0', '0', '0',       # x, y, z (co-located)
            '0', '0', '0', '1',  # qx, qy, qz, qw
            'camera_link', 'camera_depth_frame'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': '''<?xml version="1.0"?>
<robot name="x500_depth">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.47 0.47 0.11"/>
      </geometry>
      <material name="black">
        <color rgba="0.2 0.2 0.2 1.0"/>
      </material>
    </visual>
  </link>
  
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.025 0.1 0.025"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
  </link>
  
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
  </joint>
  
  <link name="camera_depth_frame"/>
  
  <joint name="camera_depth_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_depth_frame"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
</robot>'''
            }
        ],
        condition=IfCondition(LaunchConfiguration('enable_robot_state')),
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_config_arg,
        enable_rviz_arg,
        enable_robot_state_arg,
        
        rviz_node,
        base_to_camera_tf,
        camera_to_depth_tf,
        robot_state_publisher,
    ])