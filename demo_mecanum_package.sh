#!/bin/bash

# Demo script for mecanum_ros2_control package
# This script demonstrates how to use the new package

echo "======================================================"
echo "   Mecanum ROS2 Control Package Demo"
echo "======================================================"
echo
echo "This package provides a complete ROS2 Control implementation"
echo "for mecanum wheel robots, replacing the original i2c_pkg"
echo "functionality with a standardized architecture."
echo

# Package structure
echo "Package Structure:"
echo "├── src/"
echo "│   ├── mecanum_hardware_interface.cpp  # I2C hardware interface"
echo "│   └── mecanum_controller.cpp          # Unified controller"
echo "├── include/mecanum_ros2_control/"
echo "│   ├── mecanum_hardware_interface.hpp"
echo "│   └── mecanum_controller.hpp"
echo "├── config/"
echo "│   └── mecanum_controllers.yaml        # Controller parameters"
echo "├── launch/"
echo "│   ├── mecanum_robot.launch.py         # Main launch file"
echo "│   ├── mecanum_robot_simple.launch.py  # Simplified launch"
echo "│   └── test_mecanum.py                 # Test script"
echo "├── urdf/"
echo "│   └── mecanum_robot.urdf              # Robot description"
echo "└── README.md                           # Documentation"
echo

# Usage examples
echo "Usage Examples:"
echo
echo "1. Build the package:"
echo "   colcon build --packages-select mecanum_ros2_control"
echo
echo "2. Launch the robot control system:"
echo "   ros2 launch mecanum_ros2_control mecanum_robot_simple.launch.py"
echo
echo "3. Control via velocity commands:"
echo "   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \\"
echo "     \"{linear: {x: 0.2, y: 0.1}, angular: {z: 0.1}}\""
echo
echo "4. Control via position commands:"
echo "   ros2 topic pub /target_position geometry_msgs/msg/PoseStamped \\"
echo "     \"{header: {stamp: now, frame_id: 'odom'}, \\"
echo "      pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}\""
echo
echo "5. Run automated tests:"
echo "   python3 src/mecanum_ros2_control/launch/test_mecanum.py"
echo

# Key differences from original
echo "Key Improvements over Original i2c_pkg:"
echo "• Standard ROS2 Control architecture"
echo "• Modular hardware interface and controller"
echo "• YAML-based configuration"
echo "• Standard debugging tools support"
echo "• Better separation of concerns"
echo "• Easier to extend and maintain"
echo

# Topics and services
echo "Available Topics:"
echo "• /cmd_vel - Velocity control input"
echo "• /target_position - Position control input"
echo "• /odom - Robot odometry output"
echo "• /target_reached - Target reached notification"
echo "• /joint_states - Joint state information"
echo

echo "Hardware Compatibility:"
echo "• Same I2C protocol as original i2c_pkg"
echo "• Compatible with existing motor driver (0x34)"
echo "• Same encoder interface (43000 counts/rev)"
echo "• Same mecanum wheel configuration"
echo

echo "======================================================"
echo "Package is ready for testing and deployment!"
echo "======================================================"