cmake_minimum_required(VERSION 3.8)
project(mecanum_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(control_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Find i2c library
find_library(I2C_LIB i2c)

# Hardware interface
add_library(mecanum_hardware_interface SHARED
  src/mecanum_hardware_interface.cpp
)
target_include_directories(mecanum_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mecanum_hardware_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  realtime_tools
  std_msgs
  geometry_msgs
  sensor_msgs
)
target_link_libraries(mecanum_hardware_interface ${I2C_LIB})

# Mecanum controller
add_library(mecanum_controller SHARED
  src/mecanum_controller.cpp
)
target_include_directories(mecanum_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mecanum_controller
  rclcpp
  rclcpp_lifecycle
  controller_interface
  pluginlib
  realtime_tools
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2
)

# Export plugin descriptions
pluginlib_export_plugin_description_file(hardware_interface mecanum_hardware_interface_plugins.xml)
pluginlib_export_plugin_description_file(controller_interface mecanum_controller_plugins.xml)

# Install targets
install(TARGETS mecanum_hardware_interface mecanum_controller
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES mecanum_hardware_interface_plugins.xml mecanum_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()