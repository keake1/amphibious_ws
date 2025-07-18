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

    lifecycle_contoller_node = Node(
        package='lifecycle_controller',
        executable='lifecycle_controller',
        name='lifecycle_controller'
    )

    rescue_task_node = Node(
        package='activity_controller',
        executable='rescue_task_test2',
        name='rescue_task_test2'
    )

    com_node = Node(
        package='com_pkg',
        executable='com_amp',
        name='com_amp',
    )
    camera_node = Node(
        package='camera_pkg',
        executable='camera_pub_lifecycle',
        name='camera_pub_lifecycle',
        output='screen'
    )
    detect_node = Node(
        package='yolo_detect_pkg',
        executable='yolo11_detector',
        output='screen'
    )
    locate_node = Node(
        package='camera_pkg',
        executable='people_locate',
        output='screen'
    )
    temp_node = Node(
        package='temp_cam_pkg',
        executable='temp_cam_driver_lifecycle',
        output='screen'
    )
    return LaunchDescription([
        TimerAction(period = 0.0, actions=[fly_carto_launch]),
        TimerAction(period = 5.0, actions=[camera_node]),
        TimerAction(period = 6.0, actions=[detect_node]),
        TimerAction(period = 7.0, actions=[locate_node]),
        TimerAction(period = 8.0, actions=[temp_node]),
        TimerAction(period = 9.0, actions=[lifecycle_contoller_node]),
        TimerAction(period = 12.0, actions=[com_node]),
    ])