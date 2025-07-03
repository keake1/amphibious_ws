"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
import os

def generate_launch_description():
    ## ***** Launch arguments *****
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('bluesea2').find('bluesea2'), 'launch', 'uart_lidar.launch')
        )
    )

  ## ***** File paths ******
    pkg_share = FindPackageShare('my_carto_pkg').find('my_carto_pkg')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'fly.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    ## ***** Nodes *****
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': False}],
        output = 'screen'
        )

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': False}],
        arguments = [
            '-configuration_directory', FindPackageShare('my_carto_pkg').find('my_carto_pkg') + '/configuration_files',
            '-configuration_basename', 'amphi.lua'],
        remappings = [
            ('scan', 'scan')],
        output = 'screen'
        )

    com_node = Node(
        package= 'com_pkg',
        executable= 'com_node_without_imu',
        output= 'screen'
    )

    target_pub_node = Node(
        package='i2c_pkg',
        executable='target_pub',
        name='target_publisher_node',
        output='screen',
        parameters=[{
                'target_x': 0.0,
                'target_y': 0.0,
                'target_yaw': 0.0,
                'publish_rate_ms': 10
            }]
    )

    velocity_get_node = Node(
        package='i2c_pkg',
        executable='velocity_by_encoder',
        output='screen',
        parameters=[{
            'encoder_counts_per_rev': 43000,
            'wheel_diameter': 0.08,
            'i2c_bus': 0,
            'publish_counts': False,
        }]
    )

    pid_node = Node(
        package='i2c_pkg',
        executable='pid',
        output='screen',
        parameters=[{
                'position_kp': 0.75,
                'position_ki': 0.04,
                'position_kd': 0.005,
                'angle_kp': 1.3,
                'angle_ki': 20.0,
                'angle_kd': 12.0,
                'max_linear_speed': 2.0,
                'max_angular_speed': 14.0,
                'integral_limit_factor': 0.2,
                'position_integral_region': 0.3,
                'angle_integral_region': 0.5,
		'position_deadzone': 0.04,
		'angle_deadzone': 0.005,
            }]
    )

    pid_velocity_node = Node(
        package='i2c_pkg',
        executable='pid_velocity',
        output='screen',
        parameters=[{
                'kp': 0.1,
                'ki': 2.1,
                'kd': 0.0,
                'pwm_limit': 80.0,
                'deadzone': 0.03,
		'wheel_base': 0.21,
		'track_width': 0.20,
		'wheel_radius': 0.04,
		'startup_pwm': 42.0
            }]
    )

    car_drive_node = Node(
        package='i2c_pkg',
        executable='car_drive_pid_pwm',
        output='screen',
    )
    return LaunchDescription([
        # Step 1: Launch lidar_launch
        TimerAction(
            period=0.0,  # Immediately
            actions=[lidar_launch]
        ),
        # Step 2: Launch robot_state_publisher_node after 2 seconds
        TimerAction(
            period=2.0,
            actions=[robot_state_publisher_node]
        ),
        # Step 3: Launch cartographer_node after 4 seconds
        TimerAction(
            period=3.0,
            actions=[cartographer_node]
        ),
        # TimerAction(
        #     period=8.0,
        #     actions=[com_node]
        # ),
        TimerAction(
            period=5.0,
            actions=[target_pub_node]
        ),
        # TimerAction(
        #     period=5.0,
        #     actions=[velocity_get_node]
        # ),
        TimerAction(
            period=6.0,
            actions=[pid_node]
        ),
        TimerAction(
            period=6.0,
            actions=[pid_velocity_node]
        ),
        TimerAction(
            period=8.0,
            actions=[car_drive_node]
        ),

    ])
