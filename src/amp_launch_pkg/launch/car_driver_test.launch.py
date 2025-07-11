from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car_driver',
            executable='wheel_speeds_pub',
            name='wheel_speeds_pub',
            parameters=[
                {'m1_speed': 500.0},
                {'m2_speed': 500.0},
                {'m3_speed': 500.0},
                {'m4_speed': 500.0}
            ]
        ),
        Node(
            package='car_driver',
            executable='car_driver',
            name='car_driver'
        )
    ])