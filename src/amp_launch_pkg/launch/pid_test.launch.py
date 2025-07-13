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
            {'x': -1.0},
            {'y': 0.0},
            {'yaw': 0.0}
        ]
    )

    control_node = Node(
        package='pid_controller',
        executable='control_node',
        name='control_node',
        parameters=[
            {'pid_x_kp': 1.0},
            {'pid_x_ki': 0.0},
            {'pid_x_kd': 0.005},
            {'pid_x_dead_zone': 0.05},
            {'pid_x_max_output': 0.7},
            {'pid_y_kp': 1.0},
            {'pid_y_ki': 0.0},
            {'pid_y_kd': 0.005},
            {'pid_y_dead_zone': 0.05},
            {'pid_y_max_output': 0.7},
            {'pid_yaw_kp': 2.0},
            {'pid_yaw_ki': 0.0},
            {'pid_yaw_kd': 0.01},
            {'pid_yaw_dead_zone': 0.05},
            {'pid_yaw_max_output': 0.8*5},
        ]
    )

    car_driver_node = Node(
        package='car_driver',
        executable='car_driver',
        name='car_driver'
    )
    return LaunchDescription([
        TimerAction(period = 0.0, actions=[fly_carto_launch]),
        TimerAction(period = 5.0, actions=[target_pub_node]),
        TimerAction(period = 6.0, actions=[control_node]),
        TimerAction(period = 7.0, actions=[car_driver_node]),
    ])