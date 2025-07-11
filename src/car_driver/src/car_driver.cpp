#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <boost/asio.hpp>

using namespace std::chrono_literals;

class CarDriver : public rclcpp::Node {
public:
  CarDriver() : Node("car_driver"), io_(), serial_(io_) {
    this->declare_parameter("serial_port", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 115200);
    this->declare_parameter("motor_type", 2);
    this->declare_parameter("upload_data", 3);

    serial_port_ = this->get_parameter("serial_port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    motor_type_ = this->get_parameter("motor_type").as_int();
    upload_data_ = this->get_parameter("upload_data").as_int();

    if (!init_serial()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port");
      return;
    }

    wheel_speeds_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/wheel_speeds", 10, std::bind(&CarDriver::wheel_speeds_callback, this, std::placeholders::_1));

    motor_speed_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motor_speed", 10);

    init_motor_parameters();

    start_async_read();

    io_thread_ = std::thread([this]() { io_.run(); });

    RCLCPP_INFO(this->get_logger(), "Car driver initialized");
  }

  ~CarDriver() {
    control_pwm(0, 0, 0, 0);
    if (serial_.is_open()) serial_.close();
    io_.stop();
    if (io_thread_.joinable()) io_thread_.join();
  }

private:
  bool init_serial() {
    try {
      serial_.open(serial_port_);
      serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
      serial_.set_option(boost::asio::serial_port_base::character_size(8));
      serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
      serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
      return true;
    } catch (const boost::system::system_error& e) {
      RCLCPP_ERROR(this->get_logger(), "Serial error: %s", e.what());
      return false;
    }
  }

  void start_async_read() {
    serial_.async_read_some(
      boost::asio::buffer(read_buffer_),
      [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
        if (!error && bytes_transferred > 0) {
          recv_buffer_ += std::string(read_buffer_.data(), bytes_transferred);
          process_received_data();
        }
        start_async_read();
      }
    );
  }

  void process_received_data() {
    size_t pos;
    while ((pos = recv_buffer_.find('#')) != std::string::npos) {
      std::string message = recv_buffer_.substr(0, pos + 1);
      recv_buffer_ = recv_buffer_.substr(pos + 1);
      auto values = parse_data(message);
      if (!values.empty()) {
        auto msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
        msg->data = values;
        motor_speed_pub_->publish(std::move(msg));
      }
    }
  }

  void send_data(const std::string& data) {
    if (!serial_.is_open()) return;
    boost::asio::write(serial_, boost::asio::buffer(data.c_str(), data.length()));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  void set_motor_type(int type) { send_data("$mtype:" + std::to_string(type) + "#"); }
  void set_motor_deadzone(int dz) { send_data("$deadzone:" + std::to_string(dz) + "#"); }
  void set_pluse_line(int line) { send_data("$mline:" + std::to_string(line) + "#"); }
  void set_pluse_phase(int phase) { send_data("$mphase:" + std::to_string(phase) + "#"); }
  void set_wheel_dis(double wheel) { send_data("$wdiameter:" + std::to_string(wheel) + "#"); }
  void control_speed(int m1, int m2, int m3, int m4) {
    send_data("$spd:" + std::to_string(m1) + "," + std::to_string(m2) + "," +
              std::to_string(m3) + "," + std::to_string(m4) + "#");
  }
  void control_pwm(int m1, int m2, int m3, int m4) {
    send_data("$pwm:" + std::to_string(m1) + "," + std::to_string(m2) + "," +
              std::to_string(m3) + "," + std::to_string(m4) + "#");
  }
  std::vector<float> parse_data(const std::string& data) {
    std::vector<float> values;
    std::string trimmed = data;
    trimmed.erase(0, trimmed.find_first_not_of(" \n\r\t"));
    trimmed.erase(trimmed.find_last_not_of(" \n\r\t") + 1);
    if (trimmed.rfind("$MAll:", 0) == 0 || trimmed.rfind("$MTEP:", 0) == 0 || trimmed.rfind("$MSPD:", 0) == 0) {
      std::string values_str = trimmed.substr(6, trimmed.length() - 7);
      size_t pos = 0;
      std::string token;
      while ((pos = values_str.find(',')) != std::string::npos) {
        token = values_str.substr(0, pos);
        values.push_back(std::stof(token));
        values_str.erase(0, pos + 1);
      }
      if (!values_str.empty()) values.push_back(std::stof(values_str));
    }
    return values;
  }
  void send_upload_command(int mode) {
    if (mode == 0) send_data("$upload:0,0,0#");
    else if (mode == 1) send_data("$upload:1,0,0#");
    else if (mode == 2) send_data("$upload:0,1,0#");
    else if (mode == 3) send_data("$upload:0,0,1#");
  }
  void init_motor_parameters() {
    send_upload_command(upload_data_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (motor_type_ == 1) {
      set_motor_type(1); std::this_thread::sleep_for(std::chrono::milliseconds(100));
      set_pluse_phase(30); std::this_thread::sleep_for(std::chrono::milliseconds(100));
      set_pluse_line(11); std::this_thread::sleep_for(std::chrono::milliseconds(100));
      set_wheel_dis(67.00); std::this_thread::sleep_for(std::chrono::milliseconds(100));
      set_motor_deadzone(1600); std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } else if (motor_type_ == 2) {
      set_motor_type(2); std::this_thread::sleep_for(std::chrono::milliseconds(100));
      set_pluse_phase(20); std::this_thread::sleep_for(std::chrono::milliseconds(100));
      set_pluse_line(500); std::this_thread::sleep_for(std::chrono::milliseconds(100));
      set_wheel_dis(80.00); std::this_thread::sleep_for(std::chrono::milliseconds(100));
      set_motor_deadzone(1300); std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } else if (motor_type_ == 3) {
      set_motor_type(3); std::this_thread::sleep_for(std::chrono::milliseconds(100));
      set_pluse_phase(45); std::this_thread::sleep_for(std::chrono::milliseconds(100));
      set_pluse_line(13); std::this_thread::sleep_for(std::chrono::milliseconds(100));
      set_wheel_dis(68.00); std::this_thread::sleep_for(std::chrono::milliseconds(100));
      set_motor_deadzone(1250); std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } else if (motor_type_ == 4) {
      set_motor_type(4); std::this_thread::sleep_for(std::chrono::milliseconds(100));
      set_pluse_phase(48); std::this_thread::sleep_for(std::chrono::milliseconds(100));
      set_motor_deadzone(1000); std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } else if (motor_type_ == 5) {
      set_motor_type(1); std::this_thread::sleep_for(std::chrono::milliseconds(100));
      set_pluse_phase(40); std::this_thread::sleep_for(std::chrono::milliseconds(100));
      set_pluse_line(11); std::this_thread::sleep_for(std::chrono::milliseconds(100));
      set_wheel_dis(67.00); std::this_thread::sleep_for(std::chrono::milliseconds(100));
      set_motor_deadzone(1600); std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  void wheel_speeds_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 4) {
      int m1 = static_cast<int>(msg->data[0]);
      int m2 = static_cast<int>(msg->data[1]);
      int m3 = static_cast<int>(msg->data[2]);
      int m4 = static_cast<int>(msg->data[3]);
      if (motor_type_ == 4) control_pwm(m1, m2, m3, m4);
      else control_speed(-m1, -m2, -m3, -m4);
    }
  }

  std::string serial_port_;
  int baudrate_;
  int motor_type_;
  int upload_data_;
  std::string recv_buffer_;
  boost::asio::io_service io_;
  boost::asio::serial_port serial_;
  std::array<char, 256> read_buffer_;
  std::thread io_thread_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speeds_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_speed_pub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CarDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}