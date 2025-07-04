#include "mecanum_ros2_control/mecanum_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace mecanum_ros2_control
{

hardware_interface::CallbackReturn MecanumHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  logger_ = rclcpp::get_logger("MecanumHardwareInterface");

  // Initialize parameters
  encoder_counts_per_rev_ = 43000.0;  // Default value
  wheel_radius_ = 0.04;  // Default value in meters
  max_velocity_ = 50.0;  // Default max velocity value
  
  // Read parameters from URDF
  for (const auto & param : info_.hardware_parameters)
  {
    if (param.first == "encoder_counts_per_rev")
    {
      encoder_counts_per_rev_ = std::stod(param.second);
    }
    else if (param.first == "wheel_radius")
    {
      wheel_radius_ = std::stod(param.second);
    }
    else if (param.first == "max_velocity")
    {
      max_velocity_ = std::stod(param.second);
    }
  }

  // Initialize joint names and state vectors
  joint_names_.resize(info_.joints.size());
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_efforts_.resize(info_.joints.size(), 0.0);

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    joint_names_[i] = info_.joints[i].name;
  }

  // Initialize I2C
  i2c_fd_ = -1;
  first_read_ = true;
  last_encoder_counts_.fill(0);
  current_encoder_counts_.fill(0);

  RCLCPP_INFO(logger_, "Mecanum hardware interface initialized successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Configuring Mecanum hardware interface...");

  if (!init_i2c())
  {
    RCLCPP_ERROR(logger_, "Failed to initialize I2C");
    return hardware_interface::CallbackReturn::ERROR;
  }

  motor_init();

  RCLCPP_INFO(logger_, "Mecanum hardware interface configured successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MecanumHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MecanumHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MecanumHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating Mecanum hardware interface...");

  // Reset encoder counts
  if (!read_encoder_counts(current_encoder_counts_))
  {
    RCLCPP_WARN(logger_, "Failed to read initial encoder counts");
  }
  last_encoder_counts_ = current_encoder_counts_;
  first_read_ = true;

  RCLCPP_INFO(logger_, "Mecanum hardware interface activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MecanumHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating Mecanum hardware interface...");

  // Stop all motors
  std::vector<int8_t> zero_speeds = {0, 0, 0, 0};
  set_motor_speed(zero_speeds);

  RCLCPP_INFO(logger_, "Mecanum hardware interface deactivated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MecanumHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Read encoder counts
  if (!read_encoder_counts(current_encoder_counts_))
  {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared().get(), 1000, 
                         "Failed to read encoder counts");
    return hardware_interface::return_type::ERROR;
  }

  // Calculate velocities and positions
  calculate_velocities();
  
  last_read_time_ = time;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MecanumHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Convert velocity commands to motor speeds
  std::vector<int8_t> motor_speeds(4);
  
  for (size_t i = 0; i < hw_commands_.size(); ++i)
  {
    // Convert from rad/s to motor speed (-100 to 100)
    // Limit and scale the velocity command
    double limited_velocity = std::max(-max_velocity_, std::min(max_velocity_, hw_commands_[i]));
    motor_speeds[i] = static_cast<int8_t>(limited_velocity * 100.0 / max_velocity_);
  }

  // Send commands to motors
  set_motor_speed(motor_speeds);

  return hardware_interface::return_type::OK;
}

bool MecanumHardwareInterface::init_i2c()
{
  i2c_fd_ = open(I2C_BUS, O_RDWR);
  if (i2c_fd_ < 0)
  {
    RCLCPP_ERROR(logger_, "Unable to open I2C device: %s", I2C_BUS);
    return false;
  }

  if (ioctl(i2c_fd_, I2C_SLAVE, MOTOR_ADDR) < 0)
  {
    RCLCPP_ERROR(logger_, "Unable to set I2C slave address");
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }

  return true;
}

void MecanumHardwareInterface::cleanup_i2c()
{
  if (i2c_fd_ >= 0)
  {
    close(i2c_fd_);
    i2c_fd_ = -1;
  }
}

bool MecanumHardwareInterface::i2c_write_byte(uint8_t reg, uint8_t value)
{
  uint8_t buf[2] = {reg, value};
  if (write(i2c_fd_, buf, 2) != 2)
  {
    RCLCPP_ERROR(logger_, "I2C write byte failed");
    return false;
  }
  return true;
}

bool MecanumHardwareInterface::i2c_write_block(uint8_t reg, const std::vector<int8_t>& values)
{
  std::vector<uint8_t> buf(values.size() + 1);
  buf[0] = reg;
  std::memcpy(buf.data() + 1, values.data(), values.size());
  
  if (write(i2c_fd_, buf.data(), buf.size()) != static_cast<ssize_t>(buf.size()))
  {
    RCLCPP_ERROR(logger_, "I2C block write failed");
    return false;
  }
  return true;
}

bool MecanumHardwareInterface::i2c_read_block(uint8_t reg, uint8_t* values, size_t length)
{
  if (write(i2c_fd_, &reg, 1) != 1)
  {
    RCLCPP_ERROR(logger_, "I2C write register address failed");
    return false;
  }
  
  if (read(i2c_fd_, values, length) != static_cast<ssize_t>(length))
  {
    RCLCPP_ERROR(logger_, "I2C read data failed");
    return false;
  }
  return true;
}

void MecanumHardwareInterface::motor_init()
{
  // Set motor type
  i2c_write_byte(MOTOR_TYPE_ADDR, MOTOR_TYPE_JGB37_520_12V_110RPM);
  
  // Set encoder polarity (all positive)
  uint8_t encoder_polarity[4] = {0, 0, 0, 0};
  if (write(i2c_fd_, encoder_polarity, 4) != 4)
  {
    RCLCPP_WARN(logger_, "Failed to set encoder polarity");
  }
  
  RCLCPP_INFO(logger_, "Motor initialization completed");
}

void MecanumHardwareInterface::set_motor_speed(const std::vector<int8_t>& speed)
{
  if (speed.size() != 4)
  {
    RCLCPP_ERROR(logger_, "Invalid motor speed vector size");
    return;
  }
  
  if (!i2c_write_block(MOTOR_FIXED_SPEED_ADDR, speed))
  {
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock::make_shared().get(), 1000,
                         "Failed to set motor speeds");
  }
}

bool MecanumHardwareInterface::read_encoder_counts(std::array<int32_t, 4>& counts)
{
  uint8_t buffer[16];
  if (!i2c_read_block(MOTOR_ENCODER_TOTAL_ADDR, buffer, 16))
  {
    return false;
  }

  // Convert byte data to 4 int32 values
  for (int i = 0; i < 4; i++)
  {
    counts[i] = buffer[i*4] | (buffer[i*4+1] << 8) |
                (buffer[i*4+2] << 16) | (buffer[i*4+3] << 24);
  }

  return true;
}

void MecanumHardwareInterface::calculate_velocities()
{
  if (first_read_)
  {
    first_read_ = false;
    last_encoder_counts_ = current_encoder_counts_;
    return;
  }

  // Calculate wheel circumference
  double wheel_circumference = 2.0 * M_PI * wheel_radius_;

  for (size_t i = 0; i < 4; ++i)
  {
    // Calculate position difference
    int32_t delta_counts = current_encoder_counts_[i] - last_encoder_counts_[i];
    
    // Calculate position (in radians)
    double delta_position = (static_cast<double>(delta_counts) / encoder_counts_per_rev_) * 2.0 * M_PI;
    hw_positions_[i] += delta_position;
    
    // Calculate velocity (rad/s) - simplified time calculation
    // In a real implementation, you would calculate dt properly
    hw_velocities_[i] = delta_position / 0.01;  // Assuming 100Hz update rate
  }

  last_encoder_counts_ = current_encoder_counts_;
}

}  // namespace mecanum_ros2_control

PLUGINLIB_EXPORT_CLASS(
  mecanum_ros2_control::MecanumHardwareInterface, hardware_interface::SystemInterface)