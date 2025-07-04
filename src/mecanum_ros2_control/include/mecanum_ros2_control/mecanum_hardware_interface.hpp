#ifndef MECANUM_ROS2_CONTROL__MECANUM_HARDWARE_INTERFACE_HPP_
#define MECANUM_ROS2_CONTROL__MECANUM_HARDWARE_INTERFACE_HPP_

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <memory>
#include <string>
#include <chrono>

namespace mecanum_ros2_control
{
class MecanumHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MecanumHardwareInterface);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // I2C constants
  static constexpr char I2C_BUS[] = "/dev/i2c-0";
  static constexpr uint8_t MOTOR_ADDR = 0x34;
  static constexpr uint8_t MOTOR_FIXED_SPEED_ADDR = 0x33;
  static constexpr uint8_t MOTOR_ENCODER_TOTAL_ADDR = 0x3C;
  static constexpr uint8_t MOTOR_TYPE_ADDR = 0x14;
  static constexpr uint8_t MOTOR_ENCODER_POLARITY_ADDR = 0x15;
  static constexpr uint8_t MOTOR_TYPE_JGB37_520_12V_110RPM = 3;

  // I2C functionality
  bool init_i2c();
  void cleanup_i2c();
  bool i2c_write_block(uint8_t reg, const std::vector<int8_t>& values);
  bool i2c_read_block(uint8_t reg, uint8_t* values, size_t length);
  bool i2c_write_byte(uint8_t reg, uint8_t value);

  // Motor control
  void motor_init();
  void set_motor_speed(const std::vector<int8_t>& speed);
  bool read_encoder_counts(std::array<int32_t, 4>& counts);
  void calculate_velocities();

  // State variables
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Hardware parameters
  std::vector<std::string> joint_names_;
  double encoder_counts_per_rev_;
  double wheel_radius_;
  double max_velocity_;
  
  // I2C communication
  int i2c_fd_;
  
  // Encoder tracking
  std::array<int32_t, 4> last_encoder_counts_;
  std::array<int32_t, 4> current_encoder_counts_;
  rclcpp::Time last_read_time_;
  bool first_read_;
  
  // Node for logging
  rclcpp::Logger logger_;
};

}  // namespace mecanum_ros2_control

#endif  // MECANUM_ROS2_CONTROL__MECANUM_HARDWARE_INTERFACE_HPP_