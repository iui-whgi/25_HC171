#!/usr/bin/env python3
#
# Dual Arm Hardware Launch File
# 양팔 제어를 위한 별도 런치 파일
# 기존 hardware.launch.py와 완전히 독립적
#

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 패키지 경로
    pkg_path = get_package_share_directory('open_manipulator_x_bringup')
    
    # Launch 인자 선언
    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyACM0',
        description='USB port name for OpenCR'
    )
    
    # 양팔용 config 파일 사용
    config_file = os.path.join(
        pkg_path,
        'config',
        'hardware_controller_manager_dual.yaml'  # 양팔용 config
    )
    
    # Controller Manager Node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            config_file,
            {'open_manipulator_dual_hardware.port_name': LaunchConfiguration('port_name')}
        ],
        output='screen',
    )
    
    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    
    # Arm Controller Spawner (8개 조인트)
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )
    
    # Gripper Controller Spawner
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': 'dual_arm_robot_description'
        }]
    )
    
    return LaunchDescription([
        port_name_arg,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        robot_state_publisher,
    ])