cmake_minimum_required(VERSION 3.8)
project(com_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(com_node src/com_node.cpp)
ament_target_dependencies(com_node rclcpp tf2_ros tf2_geometry_msgs geometry_msgs sensor_msgs)
add_executable(com_node_without_imu src/com_node_without_imu.cpp)
ament_target_dependencies(com_node_without_imu rclcpp tf2_ros tf2_geometry_msgs geometry_msgs sensor_msgs)
add_executable(com_amp src/com_amp.cpp)
ament_target_dependencies(com_amp rclcpp tf2_ros tf2_geometry_msgs geometry_msgs sensor_msgs std_msgs)

install(TARGETS com_node com_node_without_imu com_amp
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
