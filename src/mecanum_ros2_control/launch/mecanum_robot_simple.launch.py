#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Robot description - simplified for testing
    robot_description = {
        'robot_description': open(
            os.path.join(
                os.path.dirname(__file__),
                '..',
                'urdf',
                'mecanum_robot.urdf'
            )
        ).read(),
        'use_sim_time': use_sim_time
    }
    
    # Controller configuration
    controller_config = os.path.join(
        os.path.dirname(__file__),
        '..',
        'config',
        'mecanum_controllers.yaml'
    )
    
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
        output='screen'
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