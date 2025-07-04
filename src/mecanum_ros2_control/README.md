# Mecanum ROS2 Control

This package implements a ROS2 Control framework for controlling a mecanum wheel robot, providing the same functionality as the original `i2c_pkg` but using the standard ROS2 Control architecture.

## Features

- **Hardware Interface**: I2C-based motor control compatible with the original i2c_pkg hardware
- **Mecanum Controller**: Supports both velocity and position control modes
- **PID Control**: Integrated PID controllers for precise position and orientation control
- **Encoder Feedback**: Real-time encoder reading for velocity and position feedback
- **Lifecycle Management**: Proper lifecycle management for robust operation

## Architecture

### Hardware Interface (`MecanumHardwareInterface`)
- Communicates with motors via I2C (same protocol as original i2c_pkg)
- Reads encoder values for velocity/position feedback
- Provides standard ROS2 Control interfaces

### Controller (`MecanumController`)
- Supports two control modes:
  1. **Velocity Control**: Direct cmd_vel topic control
  2. **Position Control**: Target pose following with PID control
- Publishes odometry and TF transforms
- Implements mecanum wheel kinematics

## File Structure

```
mecanum_ros2_control/
├── src/
│   ├── mecanum_hardware_interface.cpp  # Hardware interface implementation
│   └── mecanum_controller.cpp          # Controller implementation
├── include/mecanum_ros2_control/
│   ├── mecanum_hardware_interface.hpp  # Hardware interface header
│   └── mecanum_controller.hpp          # Controller header
├── config/
│   └── mecanum_controllers.yaml        # Controller configuration
├── launch/
│   ├── mecanum_robot.launch.py         # Main launch file
│   ├── mecanum_robot_simple.launch.py  # Simplified launch file
│   └── test_mecanum.py                 # Test script
├── urdf/
│   └── mecanum_robot.urdf              # Robot description
└── plugin files                       # Plugin descriptions
```

## Usage

### 1. Build the package
```bash
colcon build --packages-select mecanum_ros2_control
```

### 2. Launch the robot
```bash
ros2 launch mecanum_ros2_control mecanum_robot_simple.launch.py
```

### 3. Control the robot

**Velocity Control:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
```

**Position Control:**
```bash
ros2 topic pub /target_position geometry_msgs/msg/PoseStamped "{header: {stamp: now, frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}"
```

### 4. Run tests
```bash
python3 /path/to/test_mecanum.py
```

## Topics

### Published Topics
- `/odom` (nav_msgs/Odometry): Robot odometry
- `/target_reached` (std_msgs/Bool): Target reached notification
- `/joint_states` (sensor_msgs/JointState): Joint states

### Subscribed Topics
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/target_position` (geometry_msgs/PoseStamped): Target position commands

## Parameters

### Hardware Interface Parameters
- `encoder_counts_per_rev`: Encoder counts per revolution (default: 43000)
- `wheel_radius`: Wheel radius in meters (default: 0.04)
- `max_velocity`: Maximum velocity value (default: 50.0)

### Controller Parameters
- `wheel_base`: Distance between front and rear wheels (default: 0.21)
- `track_width`: Distance between left and right wheels (default: 0.25)
- `wheel_radius`: Wheel radius in meters (default: 0.04)
- `max_linear_speed`: Maximum linear speed (default: 2.0 m/s)
- `max_angular_speed`: Maximum angular speed (default: 20.0 rad/s)

### PID Parameters
- `position_kp`, `position_ki`, `position_kd`: Position PID gains
- `angle_kp`, `angle_ki`, `angle_kd`: Orientation PID gains
- `integral_limit_factor`: Integral limit factor (default: 0.2)
- `position_integral_region`: Position integral region (default: 0.3)
- `angle_integral_region`: Angle integral region (default: 0.5)
- `position_deadzone`: Position deadzone (default: 0.04)
- `angle_deadzone`: Angle deadzone (default: 0.05)

## Comparison with Original i2c_pkg

| Feature | Original i2c_pkg | mecanum_ros2_control |
|---------|------------------|---------------------|
| Architecture | Custom lifecycle nodes | ROS2 Control framework |
| Hardware Interface | Direct I2C communication | ROS2 Control hardware interface |
| Controller | Custom PID implementation | ROS2 Control controller |
| Configuration | Launch file parameters | YAML configuration files |
| Extensibility | Package-specific | Standard ROS2 Control ecosystem |
| Debugging | Custom logging | Standard ROS2 Control tools |

## Benefits of ROS2 Control Implementation

1. **Standardization**: Uses the standard ROS2 Control framework
2. **Modularity**: Hardware interface and controller are separate components
3. **Reusability**: Components can be reused with different hardware/controllers
4. **Tooling**: Access to standard ROS2 Control debugging and monitoring tools
5. **Ecosystem**: Compatible with other ROS2 Control packages and tools
6. **Maintainability**: Follows ROS2 Control best practices

## Hardware Requirements

- I2C motor driver board (address 0x34)
- 4 mecanum wheels with encoders
- Compatible with original i2c_pkg hardware setup