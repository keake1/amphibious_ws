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

    return LaunchDescription([
        # Step 1: Launch lidar_launch
        TimerAction(
            period=0.0,  # Immediately
            actions=[lidar_launch]
        ),
        # Step 2: Launch robot_state_publisher_node after 2 seconds
        TimerAction(
            period=1.0,
            actions=[robot_state_publisher_node]
        ),
        # Step 3: Launch cartographer_node after 4 seconds
        TimerAction(
            period=5.0,
            actions=[cartographer_node]
        ),
    ])