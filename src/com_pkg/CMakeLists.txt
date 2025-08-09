cmake_minimum_required(VERSION 3.8)
project(com_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 包含头文件目录
include_directories(include)

# 创建库
add_library(${PROJECT_NAME}_lib
  src/serial_protocol.cpp
  src/data_utils.cpp
  src/serial_com_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib 
  rclcpp 
  tf2_ros 
  tf2_geometry_msgs 
  geometry_msgs 
  std_msgs
)

# 创建可执行文件
add_executable(com_amp src/main.cpp)
target_link_libraries(com_amp ${PROJECT_NAME}_lib)

ament_target_dependencies(com_amp rclcpp tf2_ros tf2_geometry_msgs geometry_msgs std_msgs)

# 安装目标
install(TARGETS com_amp
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
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
