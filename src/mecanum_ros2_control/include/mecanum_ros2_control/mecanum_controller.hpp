#ifndef MECANUM_ROS2_CONTROL__MECANUM_CONTROLLER_HPP_
#define MECANUM_ROS2_CONTROL__MECANUM_CONTROLLER_HPP_

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <cmath>

namespace mecanum_ros2_control
{

// PID Controller class
class PIDController
{
public:
  PIDController(double kp, double ki, double kd, double min_output, double max_output,
                double integral_limit = 0.0, double deadzone = 0.0, int derivative_filter_size = 3,
                double integral_region = 0.0);

  void reset();
  double compute(double error, double dt);
  void set_gains(double kp, double ki, double kd);

private:
  double kp_, ki_, kd_;
  double min_output_, max_output_;
  double integral_limit_;
  double deadzone_;
  int derivative_filter_size_;
  double integral_region_;
  double prev_error_, integral_;
  bool first_run_;
  std::deque<double> error_history_;
};

class MecanumController : public controller_interface::ControllerInterface
{
public:
  MecanumController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  // Parameters
  std::vector<std::string> joint_names_;
  double wheel_base_;
  double track_width_;
  double wheel_radius_;
  double max_linear_speed_;
  double max_angular_speed_;
  
  // PID parameters
  double position_kp_, position_ki_, position_kd_;
  double angle_kp_, angle_ki_, angle_kd_;
  double integral_limit_factor_;
  double position_integral_region_;
  double angle_integral_region_;
  double position_deadzone_;
  double angle_deadzone_;

  // State variables
  std::vector<std::reference_wrapper<double>> joint_position_state_;
  std::vector<std::reference_wrapper<double>> joint_velocity_state_;
  std::vector<std::reference_wrapper<double>> joint_effort_state_;
  std::vector<std::reference_wrapper<double>> joint_velocity_command_;

  // Control mode
  enum class ControlMode { VELOCITY, POSITION };
  ControlMode control_mode_;

  // Position control variables
  bool has_target_;
  double target_x_, target_y_, target_yaw_;
  std::unique_ptr<PIDController> position_pid_x_;
  std::unique_ptr<PIDController> position_pid_y_;
  std::unique_ptr<PIDController> angle_pid_;

  // Odometry
  double x_, y_, yaw_;
  rclcpp::Time last_update_time_;

  // Subscriptions and Publishers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>> target_reached_pub_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Realtime buffers
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> cmd_vel_buffer_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseStamped> target_pose_buffer_;

  // Methods
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void calculate_wheel_velocities(double vx, double vy, double omega, std::vector<double>& velocities);
  void update_odometry(const rclcpp::Time& time, const rclcpp::Duration& period);
  void publish_odometry(const rclcpp::Time& time);
  void publish_transforms(const rclcpp::Time& time);
  double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q);
  bool position_control_update(const rclcpp::Time& time, const rclcpp::Duration& period);
  bool velocity_control_update(const rclcpp::Time& time, const rclcpp::Duration& period);
};

}  // namespace mecanum_ros2_control

#endif  // MECANUM_ROS2_CONTROL__MECANUM_CONTROLLER_HPP_