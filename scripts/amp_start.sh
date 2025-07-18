#!/bin/bash
echo "Start script at $(date)" >> /tmp/amp_start.log
source /opt/tros/humble/setup.bash
source ~/amphibious_ws/install/setup.bash
sleep 5
echo "Launching ROS2..." >> /tmp/exam_start.log
ros2 launch amp_launch_pkg rescue_task_test3.launch.py >> /tmp/amp_start.log 2>&1
echo "End script at $(date)" >> /tmp/amp_start.log