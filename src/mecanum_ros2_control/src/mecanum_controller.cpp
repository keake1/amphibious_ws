#include "mecanum_ros2_control/mecanum_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace mecanum_ros2_control
{

// PID Controller implementation
PIDController::PIDController(double kp, double ki, double kd, double min_output, double max_output,
                           double integral_limit, double deadzone, int derivative_filter_size,
                           double integral_region)
: kp_(kp), ki_(ki), kd_(kd), min_output_(min_output), max_output_(max_output),
  integral_limit_(integral_limit > 0 ? integral_limit : max_output),
  deadzone_(deadzone), derivative_filter_size_(derivative_filter_size),
  integral_region_(integral_region), prev_error_(0.0), integral_(0.0), first_run_(true)
{
}

void PIDController::reset()
{
  prev_error_ = 0.0;
  integral_ = 0.0;
  first_run_ = true;
  error_history_.clear();
}

double PIDController::compute(double error, double dt)
{
  if (first_run_)
  {
    first_run_ = false;
    prev_error_ = error;
    return 0.0;
  }

  // Apply deadzone
  if (std::abs(error) < deadzone_)
  {
    error = 0.0;
  }

  // Proportional term
  double proportional = kp_ * error;

  // Integral term with region control
  if (integral_region_ <= 0.0 || std::abs(error) < integral_region_)
  {
    integral_ += error * dt;
    // Clamp integral to prevent windup
    integral_ = std::max(-integral_limit_, std::min(integral_limit_, integral_));
  }
  double integral_term = ki_ * integral_;

  // Derivative term with filtering
  error_history_.push_back(error);
  if (static_cast<int>(error_history_.size()) > derivative_filter_size_)
  {
    error_history_.pop_front();
  }

  double derivative = 0.0;
  if (error_history_.size() >= 2)
  {
    double filtered_error = 0.0;
    for (const auto& e : error_history_)
    {
      filtered_error += e;
    }
    filtered_error /= error_history_.size();
    derivative = kd_ * (filtered_error - prev_error_) / dt;
  }

  prev_error_ = error;

  // Calculate output
  double output = proportional + integral_term + derivative;
  
  // Clamp output
  output = std::max(min_output_, std::min(max_output_, output));

  return output;
}

void PIDController::set_gains(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

// Mecanum Controller implementation
MecanumController::MecanumController()
: x_(0.0), y_(0.0), yaw_(0.0), has_target_(false), control_mode_(ControlMode::VELOCITY)
{
}

controller_interface::InterfaceConfiguration MecanumController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& joint_name : joint_names_)
  {
    command_interfaces_config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration MecanumController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& joint_name : joint_names_)
  {
    state_interfaces_config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    state_interfaces_config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn MecanumController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Declare and get parameters
  auto node = get_node();
  
  // Declare parameters with default values
  node->declare_parameter("joints", std::vector<std::string>());
  node->declare_parameter("wheel_base", 0.21);
  node->declare_parameter("track_width", 0.25);
  node->declare_parameter("wheel_radius", 0.04);
  node->declare_parameter("max_linear_speed", 2.0);
  node->declare_parameter("max_angular_speed", 20.0);
  node->declare_parameter("position_kp", 0.36);
  node->declare_parameter("position_ki", 0.07);
  node->declare_parameter("position_kd", 0.0);
  node->declare_parameter("angle_kp", 3.6);
  node->declare_parameter("angle_ki", 0.7);
  node->declare_parameter("angle_kd", 1.0);
  node->declare_parameter("integral_limit_factor", 0.2);
  node->declare_parameter("position_integral_region", 0.3);
  node->declare_parameter("angle_integral_region", 0.5);
  node->declare_parameter("position_deadzone", 0.04);
  node->declare_parameter("angle_deadzone", 0.05);
  
  // Get parameters
  joint_names_ = node->get_parameter("joints").as_string_array();
  wheel_base_ = node->get_parameter("wheel_base").as_double();
  track_width_ = node->get_parameter("track_width").as_double();
  wheel_radius_ = node->get_parameter("wheel_radius").as_double();
  max_linear_speed_ = node->get_parameter("max_linear_speed").as_double();
  max_angular_speed_ = node->get_parameter("max_angular_speed").as_double();

  // PID parameters
  position_kp_ = node->get_parameter("position_kp").as_double();
  position_ki_ = node->get_parameter("position_ki").as_double();
  position_kd_ = node->get_parameter("position_kd").as_double();
  angle_kp_ = node->get_parameter("angle_kp").as_double();
  angle_ki_ = node->get_parameter("angle_ki").as_double();
  angle_kd_ = node->get_parameter("angle_kd").as_double();
  integral_limit_factor_ = node->get_parameter("integral_limit_factor").as_double();
  position_integral_region_ = node->get_parameter("position_integral_region").as_double();
  angle_integral_region_ = node->get_parameter("angle_integral_region").as_double();
  position_deadzone_ = node->get_parameter("position_deadzone").as_double();
  angle_deadzone_ = node->get_parameter("angle_deadzone").as_double();

  // Initialize PID controllers
  position_pid_x_ = std::make_unique<PIDController>(
    position_kp_, position_ki_, position_kd_, 
    -max_linear_speed_, max_linear_speed_,
    max_linear_speed_ * integral_limit_factor_, position_deadzone_, 5,
    position_integral_region_);
    
  position_pid_y_ = std::make_unique<PIDController>(
    position_kp_, position_ki_, position_kd_, 
    -max_linear_speed_, max_linear_speed_,
    max_linear_speed_ * integral_limit_factor_, position_deadzone_, 5,
    position_integral_region_);
    
  angle_pid_ = std::make_unique<PIDController>(
    angle_kp_, angle_ki_, angle_kd_, 
    -max_angular_speed_, max_angular_speed_,
    max_angular_speed_ * integral_limit_factor_, angle_deadzone_, 3,
    angle_integral_region_);

  // Initialize subscribers and publishers
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&MecanumController::cmd_vel_callback, this, std::placeholders::_1));
  
  target_pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    "target_position", 10, std::bind(&MecanumController::target_pose_callback, this, std::placeholders::_1));

  odom_pub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
    get_node(), "odom", 10);
  
  target_reached_pub_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(
    get_node(), "target_reached", 10);

  // Initialize TF
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Get state and command interfaces
  joint_position_state_.clear();
  joint_velocity_state_.clear();
  joint_effort_state_.clear();
  joint_velocity_command_.clear();

  for (const auto& joint_name : joint_names_)
  {
    joint_position_state_.push_back(std::ref(
      *state_interfaces_.at(joint_name + "/" + hardware_interface::HW_IF_POSITION).get_value()));
    joint_velocity_state_.push_back(std::ref(
      *state_interfaces_.at(joint_name + "/" + hardware_interface::HW_IF_VELOCITY).get_value()));
    joint_effort_state_.push_back(std::ref(
      *state_interfaces_.at(joint_name + "/" + hardware_interface::HW_IF_EFFORT).get_value()));
    joint_velocity_command_.push_back(std::ref(
      *command_interfaces_.at(joint_name + "/" + hardware_interface::HW_IF_VELOCITY).get_value()));
  }

  // Reset odometry
  x_ = 0.0;
  y_ = 0.0;
  yaw_ = 0.0;
  last_update_time_ = get_node()->get_clock()->now();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop all wheels
  for (auto& cmd : joint_velocity_command_)
  {
    cmd.get() = 0.0;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MecanumController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Update odometry
  update_odometry(time, period);

  // Check if we have a target position for position control
  auto target_pose = target_pose_buffer_.readFromRT();
  if (target_pose && target_pose->header.stamp != rclcpp::Time(0))
  {
    control_mode_ = ControlMode::POSITION;
    has_target_ = true;
    target_x_ = target_pose->pose.position.x;
    target_y_ = target_pose->pose.position.y;
    target_yaw_ = get_yaw_from_quaternion(target_pose->pose.orientation);
  }

  bool success = false;
  if (control_mode_ == ControlMode::POSITION && has_target_)
  {
    success = position_control_update(time, period);
  }
  else
  {
    success = velocity_control_update(time, period);
  }

  // Publish odometry and transforms
  publish_odometry(time);
  publish_transforms(time);

  return success ? controller_interface::return_type::OK : controller_interface::return_type::ERROR;
}

void MecanumController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  control_mode_ = ControlMode::VELOCITY;
  cmd_vel_buffer_.writeFromNonRT(*msg);
}

void MecanumController::target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  target_pose_buffer_.writeFromNonRT(*msg);
}

void MecanumController::calculate_wheel_velocities(double vx, double vy, double omega, std::vector<double>& velocities)
{
  // Mecanum wheel inverse kinematics
  double L = wheel_base_ / 2.0;
  double W = track_width_ / 2.0;
  double rotation_factor = (L + W) * omega;
  
  // Wheel velocities based on mecanum wheel configuration
  // Order: [front_right, front_left, rear_left, rear_right]
  velocities[0] = (vx + vy + rotation_factor) / wheel_radius_;  // Front right
  velocities[1] = (vx - vy - rotation_factor) / wheel_radius_;  // Front left
  velocities[2] = (vx + vy - rotation_factor) / wheel_radius_;  // Rear left
  velocities[3] = (vx - vy + rotation_factor) / wheel_radius_;  // Rear right
}

void MecanumController::update_odometry(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  // Simple odometry calculation based on wheel velocities
  // This is a simplified approach; in practice, you'd use encoder feedback
  
  double dt = period.seconds();
  
  // Get current wheel velocities
  std::vector<double> wheel_velocities(4);
  for (size_t i = 0; i < 4; ++i)
  {
    wheel_velocities[i] = joint_velocity_state_[i].get();
  }
  
  // Forward kinematics to get robot velocity
  double L = wheel_base_ / 2.0;
  double W = track_width_ / 2.0;
  
  double vx = wheel_radius_ * (wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2] + wheel_velocities[3]) / 4.0;
  double vy = wheel_radius_ * (wheel_velocities[0] - wheel_velocities[1] + wheel_velocities[2] - wheel_velocities[3]) / 4.0;
  double omega = wheel_radius_ * (wheel_velocities[0] - wheel_velocities[1] - wheel_velocities[2] + wheel_velocities[3]) / (4.0 * (L + W));
  
  // Update pose
  x_ += (vx * cos(yaw_) - vy * sin(yaw_)) * dt;
  y_ += (vx * sin(yaw_) + vy * cos(yaw_)) * dt;
  yaw_ += omega * dt;
  
  // Normalize yaw
  while (yaw_ > M_PI) yaw_ -= 2.0 * M_PI;
  while (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;
}

void MecanumController::publish_odometry(const rclcpp::Time& time)
{
  if (odom_pub_->trylock())
  {
    auto& odom_msg = odom_pub_->msg_;
    odom_msg.header.stamp = time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    
    // Position
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    
    // Orientation
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);
    
    // Velocity (simplified)
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;
    
    odom_pub_->unlockAndPublish();
  }
}

void MecanumController::publish_transforms(const rclcpp::Time& time)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = time;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  
  transform.transform.translation.x = x_;
  transform.transform.translation.y = y_;
  transform.transform.translation.z = 0.0;
  
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_);
  transform.transform.rotation = tf2::toMsg(q);
  
  tf_broadcaster_->sendTransform(transform);
}

double MecanumController::get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
{
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  tf2::Matrix3x3 m(tf_q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

bool MecanumController::position_control_update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  double dt = period.seconds();
  
  // Calculate errors
  double error_x = target_x_ - x_;
  double error_y = target_y_ - y_;
  double error_yaw = target_yaw_ - yaw_;
  
  // Normalize angle error
  while (error_yaw > M_PI) error_yaw -= 2.0 * M_PI;
  while (error_yaw < -M_PI) error_yaw += 2.0 * M_PI;
  
  // PID control
  double vx = position_pid_x_->compute(error_x, dt);
  double vy = position_pid_y_->compute(error_y, dt);
  double omega = angle_pid_->compute(error_yaw, dt);
  
  // Calculate wheel velocities
  std::vector<double> wheel_velocities(4);
  calculate_wheel_velocities(vx, vy, omega, wheel_velocities);
  
  // Apply commands
  for (size_t i = 0; i < 4; ++i)
  {
    joint_velocity_command_[i].get() = wheel_velocities[i];
  }
  
  // Check if target reached
  double distance_error = std::sqrt(error_x * error_x + error_y * error_y);
  static int arrived_count = 0;
  if (distance_error < 0.05 && std::abs(error_yaw) < 0.1)
  {
    arrived_count++;
    if (arrived_count > 50)  // 0.5 seconds at 100Hz
    {
      if (target_reached_pub_->trylock())
      {
        target_reached_pub_->msg_.data = true;
        target_reached_pub_->unlockAndPublish();
      }
      has_target_ = false;
      arrived_count = 0;
    }
  }
  else
  {
    arrived_count = 0;
  }
  
  return true;
}

bool MecanumController::velocity_control_update(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  auto cmd_vel = cmd_vel_buffer_.readFromRT();
  if (!cmd_vel)
  {
    // Stop all wheels if no command
    for (auto& cmd : joint_velocity_command_)
    {
      cmd.get() = 0.0;
    }
    return true;
  }
  
  double vx = cmd_vel->linear.x;
  double vy = cmd_vel->linear.y;
  double omega = cmd_vel->angular.z;
  
  // Calculate wheel velocities
  std::vector<double> wheel_velocities(4);
  calculate_wheel_velocities(vx, vy, omega, wheel_velocities);
  
  // Apply commands
  for (size_t i = 0; i < 4; ++i)
  {
    joint_velocity_command_[i].get() = wheel_velocities[i];
  }
  
  return true;
}

}  // namespace mecanum_ros2_control

PLUGINLIB_EXPORT_CLASS(
  mecanum_ros2_control::MecanumController, controller_interface::ControllerInterface)