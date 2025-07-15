cmake_minimum_required(VERSION 3.8)
project(camera_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED zbar)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(camera_pub src/camera_pub.cpp)
ament_target_dependencies(camera_pub rclcpp sensor_msgs cv_bridge)
target_link_libraries(camera_pub ${OpenCV_LIBS})

add_executable(camera_pub_lifecycle src/camera_pub_lifecycle.cpp)
ament_target_dependencies(camera_pub_lifecycle rclcpp sensor_msgs cv_bridge OpenCV rclcpp_lifecycle)
target_link_libraries(camera_pub_lifecycle ${OpenCV_LIBS})

add_executable(fov_measure src/fov_measure.cpp)
ament_target_dependencies(fov_measure rclcpp sensor_msgs cv_bridge std_msgs)
target_link_libraries(fov_measure ${OpenCV_LIBS} ${ZBAR_LIBRARIES})

install(TARGETS camera_pub camera_pub_lifecycle fov_measure
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
