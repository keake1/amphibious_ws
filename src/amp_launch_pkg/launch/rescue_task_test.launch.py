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

    control_node = Node(
        package='pid_controller',
        executable='control_node_lifecycle',
        name='control_node_lifecycle',
        parameters=[
            {'pid_x_kp': 0.8},
            {'pid_x_ki': 0.03},
            {'pid_x_kd': 0.0},
            {'pid_x_dead_zone': 0.05},
            {'pid_x_max_output': 0.8},
            {'pid_y_kp': 0.8},
            {'pid_y_ki': 0.03},
            {'pid_y_kd': 0.0},
            {'pid_y_dead_zone': 0.05},
            {'pid_y_max_output': 0.8},
            {'pid_yaw_kp': 1.5},
            {'pid_yaw_ki': 0.0},
            {'pid_yaw_kd': 0.05},
            {'pid_yaw_dead_zone': 0.05},
            {'pid_yaw_max_output': 0.8*5},
        ]
    )

    car_driver_node = Node(
        package='car_driver',
        executable='car_driver',
        name='car_driver'
    )

    lifecycle_contoller_node = Node(
        package='lifecycle_controller',
        executable='lifecycle_controller',
        name='lifecycle_controller'
    )

    rescue_task_node = Node(
        package='activity_controller',
        executable='rescue_task_test',
        name='rescue_task_test'
    )

    return LaunchDescription([
        TimerAction(period = 0.0, actions=[fly_carto_launch]),
        TimerAction(period = 10.0, actions=[control_node]),
        TimerAction(period = 11.0, actions=[car_driver_node]),
        TimerAction(period = 12.0, actions=[lifecycle_contoller_node]),
        TimerAction(period = 15.0, actions=[rescue_task_node]),
    ])