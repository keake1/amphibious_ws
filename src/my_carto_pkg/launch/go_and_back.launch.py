from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('my_carto_pkg').find('my_carto_pkg')

    # 启动 go_and_back 节点
    go_and_back_node = Node(
        package='car_move_pkg',
        executable='go_and_back',
        name='go_and_back_node',
        output='screen'
    )

    # 启动 com_amp 节点
    com_amp_node = Node(
        package='com_pkg',
        executable='com_amp',
        name='com_amp_node',
        output='screen'
    )

    # 包含 fly_carto.launch.py
    fly_carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'fly_carto.launch.py')
        )
    )

    return LaunchDescription([
        TimerAction(period=0.0,actions=[fly_carto_launch]),
        TimerAction(period=5.0,actions=[com_amp_node]),  
        # TimerAction(period=6.0, actions=[go_and_back_node]),
    ])