cmake_minimum_required(VERSION 3.8)
project(i2c_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_library(I2C_LIB i2c)
find_package(std_srvs REQUIRED)
find_package(amp_interfaces REQUIRED)


add_executable(car_drive src/car_drive.cpp)
ament_target_dependencies(car_drive rclcpp)
target_link_libraries(car_drive ${I2C_LIB})
add_executable(pid src/pid.cpp)
ament_target_dependencies(pid rclcpp std_msgs geometry_msgs tf2_ros tf2)
add_executable(car_drive_pid src/car_drive_pid.cpp)
ament_target_dependencies(car_drive_pid rclcpp std_msgs)
target_link_libraries(car_drive_pid ${I2C_LIB})
add_executable(target_pub src/target_pub.cpp)
ament_target_dependencies(target_pub rclcpp geometry_msgs tf2 std_srvs amp_interfaces)
add_executable(pid_velocity src/pid_velocity.cpp)
ament_target_dependencies(pid_velocity rclcpp std_msgs geometry_msgs tf2)
add_executable(car_drive_pid_pwm src/car_drive_pid_pwm.cpp)
ament_target_dependencies(car_drive_pid_pwm rclcpp std_msgs geometry_msgs tf2)
target_link_libraries(car_drive_pid_pwm ${I2C_LIB})
add_executable(pid_lifecycle src/pid_lifecycle.cpp)
ament_target_dependencies(pid_lifecycle rclcpp rclcpp_lifecycle std_msgs geometry_msgs tf2_ros tf2 lifecycle_msgs std_srvs amp_interfaces)
add_executable(pid_velocity_lifecycle src/pid_velocity_lifecycle.cpp)
ament_target_dependencies(pid_velocity_lifecycle rclcpp rclcpp_lifecycle std_msgs geometry_msgs tf2 lifecycle_msgs)
add_executable(car_drive_pid_pwm_lifecycle src/car_drive_pid_pwm_lifecycle.cpp)
ament_target_dependencies(car_drive_pid_pwm_lifecycle rclcpp rclcpp_lifecycle std_msgs geometry_msgs tf2 lifecycle_msgs)
target_link_libraries(car_drive_pid_pwm_lifecycle ${I2C_LIB})
add_executable(velocity_by_encoder src/velocity_by_encoder.cpp)
ament_target_dependencies(velocity_by_encoder rclcpp std_msgs geometry_msgs)
target_link_libraries(velocity_by_encoder ${I2C_LIB})
add_executable(velocity_by_encoder_lifecycle src/velocity_by_encoder_lifecycle.cpp)
ament_target_dependencies(velocity_by_encoder_lifecycle rclcpp std_msgs geometry_msgs rclcpp_lifecycle lifecycle_msgs)
target_link_libraries(velocity_by_encoder_lifecycle ${I2C_LIB})

install(TARGETS
  car_drive
  pid
  car_drive_pid
  target_pub
  pid_velocity
  car_drive_pid_pwm
  pid_lifecycle
  pid_velocity_lifecycle
  car_drive_pid_pwm_lifecycle
  velocity_by_encoder
  velocity_by_encoder_lifecycle
  DESTINATION lib/${PROJECT_NAME})
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

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
