from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, ExecuteProcess
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # 1. 定义摄像头发布节点
    camera_pub_node = Node(
        package='camera_pkg',
        executable='camera_pub_lifecycle',
        name='camera_pub_lifecycle_node',
        output='screen'
    )
    
    # 2. 定义YOLO目标检测生命周期节点
    yolo_detect_node = Node(
        package='yolo_detect_pkg',
        executable='yolo11_detect',
        name='yolo_detect_node',
        output='screen',
        parameters=[{
            'model_path': '/home/sunrise/rdk_model_zoo/demos/detect/YOLO11/YOLO11-Detect_YUV420SP/ptq_models/yolo11m_detect_bayese_640x640_nv12_modified.bin',
            'classes_num': 80,
            'nms_thres': 0.7,
            'score_thres': 0.4,
            'reg': 16
        }]
    )
    
    # 3. 定义物体定位节点
    item_locate_node = Node(
        package='item_locate_pkg',
        executable='item_locate_node',
        name='item_locate_node',
        output='screen',
        parameters=[{
            'camera_fov_degrees': 59.5,
            'camera_width': 640,
            'camera_height': 640,
            'camera_offset_x': 0.0,
            'camera_offset_y': 0.0
        }]
    )
    
    # 返回所有操作的组合
    return LaunchDescription([
        TimerAction(
            period=0.0, 
            actions=[camera_pub_node]
        ),
        TimerAction(
            period=1.0,
            actions=[yolo_detect_node]
        ),
        TimerAction(
            period=2.0,
            actions=[item_locate_node]
        ),
    ])