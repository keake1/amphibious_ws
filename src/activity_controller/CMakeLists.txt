cmake_minimum_required(VERSION 3.8)
project(activity_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(amp_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(rescue_task_test src/rescue_task_test.cpp)
ament_target_dependencies(rescue_task_test rclcpp std_msgs amp_interfaces tf2_ros)

add_executable(rescue_task_test2 src/rescue_task_test2.cpp)
ament_target_dependencies(rescue_task_test2 rclcpp std_msgs amp_interfaces tf2_ros)

add_executable(rescue_task_test4 src/rescue_task_test4.cpp)
ament_target_dependencies(rescue_task_test4 rclcpp std_msgs amp_interfaces tf2_ros geometry_msgs)

add_executable(rescue_task_test5 src/rescue_task_test5.cpp)
ament_target_dependencies(rescue_task_test5 rclcpp std_msgs amp_interfaces tf2_ros geometry_msgs)


install(TARGETS rescue_task_test rescue_task_test2 rescue_task_test4 rescue_task_test5
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
