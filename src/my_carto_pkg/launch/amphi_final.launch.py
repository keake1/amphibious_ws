from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_share = FindPackageShare('my_carto_pkg').find('my_carto_pkg')
    i2c_pkg_share = FindPackageShare('i2c_pkg').find('i2c_pkg')
    item_locate_pkg_share = FindPackageShare('item_locate_pkg').find('item_locate_pkg')

    # 包含 car_drive.launch.py
    car_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(i2c_pkg_share, 'launch', 'car_drive.launch.py')
        )
    )

    # 包含 item_locate.launch.py
    item_locate_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(item_locate_pkg_share, 'launch', 'item_locate.launch.py')
        )
    )

    # 包含 fly_carto.launch.py
    fly_carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'fly_carto.launch.py')
        )
    )

    lifecycle_control_node = Node(
        package='lifecycle_control_pkg',
        executable='lifecycle_control_node',
        output='screen'
    )

    com_node = Node(
        package='com_pkg',
        executable='com_amp',
        output='screen'
    )

    target_pub_node = Node(
    package='i2c_pkg',
    executable='target_pub',
    name='target_publisher_node',
    output='screen',
    parameters=[{
            'publish_rate_ms': 5,
            'waypoints': [
                '0.0,1.88,0.0',
                '0.0,0.0,0.0'
            ],
            'loop_waypoints': False,
            'auto_start': True
        }]
    )

    return LaunchDescription([
        TimerAction(
            period=0.0, 
            actions=[fly_carto_launch]
        ),
        TimerAction(
            period=12.0,
            actions=[com_node]
        ),
        TimerAction(
            period=2.0,
            actions=[lifecycle_control_node]
        ),
        TimerAction(
            period=4.0,
            actions=[car_drive_launch]
        ),
        TimerAction(
            period=6.0,
            actions=[item_locate_launch]
        ),
        TimerAction(
            period=8.0,
            actions=[target_pub_node]
        )
    ])