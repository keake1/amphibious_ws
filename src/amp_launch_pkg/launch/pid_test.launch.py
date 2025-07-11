from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    carto_pkg_path = get_package_share_directory('my_carto_pkg')
    fly_carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                carto_pkg_path,
                'launch',
                'fly_carto.launch.py'
            )
        )
    )
    target_pub_node = Node(
            package='pid_controller',
            executable='target_pub',
            name='target_pub',
            parameters=[
                {'x': 1.0},
                {'y': 0.0},
                {'yaw': 0.0}
            ]
        )

        control_node = Node(
            package='pid_controller',
            executable='control_node',
            name='control_node'
        )

        car_driver_node = Node(
            package='car_driver',
            executable='car_driver',
            name='car_driver',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},
                {'baudrate': 115200},
                {'motor_type': 2},
                {'upload_data': 3}
            ]
        ),
    return LaunchDescription([
        TimerAction(period = 0.0, actions=[fly_carto_launch]),
        TimerAction(period = 5.0, actions=[target_pub_node]),
        TimerAction(period = 6.0, actions=[control_node]),
        TimerAction(period = 7.0, actions=[car_driver_node]),
    ])