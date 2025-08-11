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
            {'pid_x_kp': 0.45},
            {'pid_x_ki': 0.02},
            {'pid_x_kd': 0.0},
            {'pid_x_dead_zone': 0.05},
            {'pid_x_max_output': 0.45},
            {'pid_y_kp': 0.45},
            {'pid_y_ki': 0.02},
            {'pid_y_kd': 0.0},
            {'pid_y_dead_zone': 0.05},
            {'pid_y_max_output': 0.35},
            {'pid_yaw_kp': 1.5},
            {'pid_yaw_ki': 0.0},
            {'pid_yaw_kd': 0.01},
            {'pid_yaw_dead_zone': 0.05},
            {'pid_yaw_max_output': 0.5*5},
        ]
    )

    car_driver_node = Node(
        package='car_driver',
        executable='car_driver',
        name='car_driver'
    )

    lifecycle_contoller_node = Node(
        package='lifecycle_controller',
        executable='lifecycle_controller_ver2',
        name='lifecycle_controller_ver2'
    )

    rescue_task_node = Node(
        package='activity_controller',
        executable='rescue_task_test4',
        name='rescue_task_test4',
        parameters=[
            {'target_reached_threshold': 0.5},
            {"position_tolerance": 0.1},
            {"angle_tolerance": 0.08},
            ],
    )

    com_node = Node(
        package='com_pkg',
        executable='com_amp',
        name='com_amp',
    )

    camera_node = Node(
        package='camera_pkg',
        executable='camera_pub',
        name='camera_pub',
        output='screen'
    )

    detect_node = Node(
        package='yolo_detect_pkg',
        executable='yolo11_detector',
        output='screen'
    )

    temp_node = Node(
        package='temp_cam_pkg',
        executable='temp_cam_driver',
        output='screen'
    )

    return LaunchDescription([
        TimerAction(period = 0.0, actions=[fly_carto_launch]),
        TimerAction(period = 5.0, actions=[camera_node]),
        TimerAction(period = 6.0, actions=[detect_node]),
        TimerAction(period = 7.0, actions=[temp_node]),
        TimerAction(period = 8.0, actions=[control_node]),
        TimerAction(period = 9.0, actions=[car_driver_node]),
        TimerAction(period = 10.0, actions=[lifecycle_contoller_node]),
        TimerAction(period = 12.0, actions=[rescue_task_node]),
        TimerAction(period = 15.0, actions=[com_node]),
    ])