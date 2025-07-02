from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, ExecuteProcess
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 创建三个生命周期节点
    car_drive_pwm_node = Node(
        package='i2c_pkg',
        executable='car_drive_pid_pwm_lifecycle',
        name='car_drive_pid_pwm_lifecycle_node',
        parameters=[{
            'motor_addr': 52,  # 0x34 十六进制转十进制
        }],
        output='screen',
    )
    
    car_pid_node = Node(
        package='i2c_pkg',
        executable='pid_lifecycle',
        name='car_pid_lifecycle_node',
        parameters=[{
            'position_kp': 0.31,
                'position_ki': 0.031,
                'position_kd': 0.0,
                'angle_kp': 4.0,
                'angle_ki': 0.4,
                'angle_kd': 0.0,
                'max_linear_speed': 2.0,
                'max_angular_speed': 20.0,
                'integral_limit_factor': 0.28,
                'position_integral_region': 0.4,  # 移动积分区域
                'angle_integral_region': 0.5,  # 角度积分区域
        }],
        output='screen',
    )
    
    velocity_pid_node = Node(
        package='i2c_pkg',
        executable='pid_velocity_lifecycle',
        name='velocity_pid_lifecycle_node',
        parameters=[{
            'kp': 0.5,
            'ki': 5.0,
            'kd': 0.0,
            'pwm_limit': 70.0,
            'deadzone': 0.01,
            'startup_pwm': 20.0
        }],
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

    # 配置状态转换事件
    configure_car_drive_pwm = RegisterEventHandler(
        OnProcessStart(
            target_action=car_drive_pwm_node,
            on_start=[
                TimerAction(
                    period=2.0,  # 等待节点完全启动
                    actions=[
                        Node(
                            package='lifecycle_msgs',
                            executable='lifecycle_manager',
                            name='lifecycle_manager_car_drive_pwm',
                            parameters=[{
                                'node_name': 'car_drive_pid_pwm_lifecycle_node',
                                'target_state': '3',  # TRANSITION_CONFIGURE
                            }],
                            output='screen',
                        ),
                    ],
                ),
            ],
        )
    )
    
    configure_car_pid = RegisterEventHandler(
        OnProcessStart(
            target_action=car_pid_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[
                        ExecuteProcess(
                            cmd=['ros2', 'lifecycle', 'set', 'car_pid_lifecycle_node', 'configure'],
                            output='screen'
                        )
                    ],
                ),
            ],
        )
    )
    
    configure_velocity_pid = RegisterEventHandler(
        OnProcessStart(
            target_action=velocity_pid_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[
                        ExecuteProcess(
                            cmd=['ros2', 'lifecycle', 'set', 'velocity_pid_lifecycle_node', 'configure'],
                            output='screen'
                        )
                    ],
                ),
            ],
        )
    )
    
    configure_velocity_get = RegisterEventHandler(
        OnProcessStart(
            target_action=velocity_get_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[
                        ExecuteProcess(
                            cmd=['ros2', 'lifecycle', 'set', 'encoder_velocity_lifecycle_node', 'configure'],
                            output='screen'
                        )
                    ],
                ),
            ],
        )
    )
    
    # 返回LaunchDescription
    return LaunchDescription([
        # 节点启动
        car_drive_pwm_node,
        car_pid_node,
        velocity_pid_node,
        # velocity_get_node,
        # 节点配置
        # configure_car_drive_pwm,
        # configure_car_pid,
        # configure_velocity_pid,
        # configure_velocity_get,
    ])