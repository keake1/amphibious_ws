from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, ExecuteProcess
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取参数文件路径
    pkg_share = get_package_share_directory('i2c_pkg')
    params_file = os.path.join(pkg_share, 'params', 'car_drive.yaml')
    
    # 创建三个生命周期节点
    car_drive_pwm_node = Node(
        package='i2c_pkg',
        executable='car_drive_pid_pwm_lifecycle',
        name='car_drive_pid_pwm_lifecycle_node',
        parameters=[params_file],
        output='screen',
    )
    
    car_pid_node = Node(
        package='i2c_pkg',
        executable='pid_lifecycle',
        name='pid_lifecycle_node',
        parameters=[params_file],
        output='screen',
    )
    
    velocity_pid_node = Node(
        package='i2c_pkg',
        executable='pid_velocity_lifecycle',
        name='velocity_pid_lifecycle_node',
        parameters=[params_file],
        output='screen',
    )
    
    velocity_get_node = Node(
        package = 'i2c_pkg',
        executable='velocity_by_encoder_lifecycle',
        name='encoder_velocity_lifecycle_node',  # 添加明确的名称
        parameters=[{
            'encoder_counts_per_rev': 43000,
            'wheel_diameter': 0.08,
            'i2c_bus': 0,
            'publish_counts': False,
        }]
    )

    
    # 返回LaunchDescription
    return LaunchDescription([
        # 节点启动
        TimerAction(
            period = 2.0,
            actions = [car_drive_pwm_node]
        ),
        TimerAction(
            period = 0.0,
            actions = [car_pid_node]
        ),
        TimerAction(
            period = 1.0,
            actions = [velocity_pid_node]
        ),
        # velocity_get_node,
    ])