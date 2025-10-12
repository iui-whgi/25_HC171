#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyACM0',
            description='USB port name for the dual arm system'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Start robot with fake hardware mirroring command to its states'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Start RViz2 with the robot model'
        )
    )

    # Initialize Arguments
    port_name = LaunchConfiguration('port_name')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    start_rviz = LaunchConfiguration('start_rviz')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('open_manipulator_x_description'),
                 'urdf', 'open_manipulator_x_dual.urdf.xacro']
            ),
            ' ',
            'use_fake_hardware:=', use_fake_hardware,
            ' ',
            'port_name:=', port_name,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Controller configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('open_manipulator_x_bringup'),
            'config',
            'dual_arm_controller.yaml',
        ]
    )

    # Control node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Left arm controller spawner
    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Right arm controller spawner
    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Left gripper controller spawner
    left_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Right gripper controller spawner
    right_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Delay controller spawners after control_node
    delay_joint_state_broadcaster_spawner_after_control_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_left_arm_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_arm_controller_spawner],
        )
    )

    delay_right_arm_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[right_arm_controller_spawner],
        )
    )

    delay_left_gripper_controller_spawner_after_left_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_arm_controller_spawner,
            on_exit=[left_gripper_controller_spawner],
        )
    )

    delay_right_gripper_controller_spawner_after_right_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_arm_controller_spawner,
            on_exit=[right_gripper_controller_spawner],
        )
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('open_manipulator_x_description'), 'rviz', 'open_manipulator_x.rviz']
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(start_rviz),
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        delay_joint_state_broadcaster_spawner_after_control_node,
        delay_left_arm_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_right_arm_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_left_gripper_controller_spawner_after_left_arm_controller_spawner,
        delay_right_gripper_controller_spawner_after_right_arm_controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)