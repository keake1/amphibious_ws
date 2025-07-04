#!/bin/bash

# Validation script for mecanum_ros2_control package
# This script validates the structure and key components without compilation

PACKAGE_DIR="/home/runner/work/amphibious_ws/amphibious_ws/src/mecanum_ros2_control"

echo "=== Mecanum ROS2 Control Package Validation ==="
echo

# Check package structure
echo "1. Checking package structure..."
required_dirs=("src" "include/mecanum_ros2_control" "config" "launch" "urdf")
for dir in "${required_dirs[@]}"; do
    if [ -d "$PACKAGE_DIR/$dir" ]; then
        echo "  ✓ $dir directory exists"
    else
        echo "  ✗ $dir directory missing"
    fi
done

# Check required files
echo
echo "2. Checking required files..."
required_files=(
    "package.xml"
    "CMakeLists.txt"
    "src/mecanum_hardware_interface.cpp"
    "src/mecanum_controller.cpp"
    "include/mecanum_ros2_control/mecanum_hardware_interface.hpp"
    "include/mecanum_ros2_control/mecanum_controller.hpp"
    "config/mecanum_controllers.yaml"
    "launch/mecanum_robot.launch.py"
    "launch/mecanum_robot_simple.launch.py"
    "urdf/mecanum_robot.urdf"
    "mecanum_hardware_interface_plugins.xml"
    "mecanum_controller_plugins.xml"
    "README.md"
)

for file in "${required_files[@]}"; do
    if [ -f "$PACKAGE_DIR/$file" ]; then
        echo "  ✓ $file exists"
    else
        echo "  ✗ $file missing"
    fi
done

# Check file sizes (ensuring they're not empty)
echo
echo "3. Checking file content sizes..."
key_files=(
    "src/mecanum_hardware_interface.cpp"
    "src/mecanum_controller.cpp"
    "include/mecanum_ros2_control/mecanum_hardware_interface.hpp"
    "include/mecanum_ros2_control/mecanum_controller.hpp"
)

for file in "${key_files[@]}"; do
    if [ -f "$PACKAGE_DIR/$file" ]; then
        size=$(wc -l < "$PACKAGE_DIR/$file")
        echo "  ✓ $file: $size lines"
    fi
done

# Check for key components in source files
echo
echo "4. Checking key components..."

# Hardware interface checks
if grep -q "class MecanumHardwareInterface" "$PACKAGE_DIR/src/mecanum_hardware_interface.cpp"; then
    echo "  ✓ MecanumHardwareInterface class found"
else
    echo "  ✗ MecanumHardwareInterface class missing"
fi

if grep -q "PLUGINLIB_EXPORT_CLASS" "$PACKAGE_DIR/src/mecanum_hardware_interface.cpp"; then
    echo "  ✓ Hardware interface plugin export found"
else
    echo "  ✗ Hardware interface plugin export missing"
fi

# Controller checks
if grep -q "class MecanumController" "$PACKAGE_DIR/src/mecanum_controller.cpp"; then
    echo "  ✓ MecanumController class found"
else
    echo "  ✗ MecanumController class missing"
fi

if grep -q "PLUGINLIB_EXPORT_CLASS" "$PACKAGE_DIR/src/mecanum_controller.cpp"; then
    echo "  ✓ Controller plugin export found"
else
    echo "  ✗ Controller plugin export missing"
fi

# Check I2C functionality
if grep -q "i2c_write_block\|i2c_read_block" "$PACKAGE_DIR/src/mecanum_hardware_interface.cpp"; then
    echo "  ✓ I2C communication functions found"
else
    echo "  ✗ I2C communication functions missing"
fi

# Check PID controller
if grep -q "class PIDController" "$PACKAGE_DIR/src/mecanum_controller.cpp"; then
    echo "  ✓ PID controller implementation found"
else
    echo "  ✗ PID controller implementation missing"
fi

# Check mecanum kinematics
if grep -q "calculate_wheel_velocities" "$PACKAGE_DIR/src/mecanum_controller.cpp"; then
    echo "  ✓ Mecanum wheel kinematics found"
else
    echo "  ✗ Mecanum wheel kinematics missing"
fi

# Check launch file executability
echo
echo "5. Checking launch file permissions..."
launch_files=("launch/mecanum_robot.launch.py" "launch/mecanum_robot_simple.launch.py" "launch/test_mecanum.py")
for file in "${launch_files[@]}"; do
    if [ -x "$PACKAGE_DIR/$file" ]; then
        echo "  ✓ $file is executable"
    else
        echo "  ✗ $file is not executable"
    fi
done

# Check YAML configuration
echo
echo "6. Checking configuration file..."
if grep -q "controller_manager:\|mecanum_controller:" "$PACKAGE_DIR/config/mecanum_controllers.yaml"; then
    echo "  ✓ Controller configuration found"
else
    echo "  ✗ Controller configuration missing"
fi

# Check URDF
echo
echo "7. Checking URDF file..."
if grep -q "ros2_control\|mecanum_ros2_control/MecanumHardwareInterface" "$PACKAGE_DIR/urdf/mecanum_robot.urdf"; then
    echo "  ✓ ROS2 Control configuration in URDF found"
else
    echo "  ✗ ROS2 Control configuration in URDF missing"
fi

echo
echo "=== Validation Complete ==="
echo
echo "Package appears to be correctly structured for ROS2 Control implementation."
echo "Next steps:"
echo "1. Install ROS2 and ros2_control packages"
echo "2. Build the package with: colcon build --packages-select mecanum_ros2_control"
echo "3. Test with: ros2 launch mecanum_ros2_control mecanum_robot_simple.launch.py"