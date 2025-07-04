#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('mecanum_ros2_control'),
            'urdf',
            'mecanum_robot.urdf'
        ])
    ])
    
    robot_description = {
        'robot_description': robot_description_content,
        'use_sim_time': use_sim_time
    }
    
    # Controller configuration
    controller_config = PathJoinSubstitution([
        FindPackageShare('mecanum_ros2_control'),
        'config',
        'mecanum_controllers.yaml'
    ])
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )
    
    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen',
        remappings=[
            ('~/robot_description', '/robot_description'),
        ]
    )
    
    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen'
    )
    
    # Mecanum controller spawner
    mecanum_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_controller', '-c', '/controller_manager'],
        output='screen'
    )
    
    # Delay spawning controllers after controller manager starts
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[joint_state_broadcaster_spawner]
                )
            ]
        )
    )
    
    delay_mecanum_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[mecanum_controller_spawner]
                )
            ]
        )
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # Nodes
        robot_state_publisher,
        controller_manager,
        
        # Event handlers
        delay_joint_state_broadcaster,
        delay_mecanum_controller,
    ])