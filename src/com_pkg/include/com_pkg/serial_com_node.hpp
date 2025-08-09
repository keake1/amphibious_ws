#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <memory>

#include "com_pkg/serial_protocol.hpp"
#include "com_pkg/data_utils.hpp"

namespace com_pkg {

class SerialComNode : public rclcpp::Node {
public:
    SerialComNode();
    ~SerialComNode();

private:
    // 初始化方法
    void initialize_serial_port();
    void initialize_ros_components();
    void initialize_timers();
    
    // 串口处理
    void start_async_read();
    void handle_read(const boost::system::error_code& error, std::size_t bytes_transferred);
    
    // 数据处理
    void process_received_frame();
    void process_mode_command(uint8_t mode_command);
    void process_height_data(const std::vector<uint8_t>& data);
    void process_voltage_data(const std::vector<uint8_t>& data);
    void process_temperature_position(const std::vector<uint8_t>& data);
    
    // 数据发送
    void send_serial_data(uint8_t type, const std::vector<uint8_t>& payload);
    void send_tf_data();
    void send_position_data(const geometry_msgs::msg::TransformStamped& transform);
    void send_orientation_data(const geometry_msgs::msg::TransformStamped& transform);
    void send_velocity_data(const geometry_msgs::msg::Twist& twist);
    
    // ROS回调函数
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void mode_switch_callback(const std_msgs::msg::String::SharedPtr msg);
    void temperature_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void duoji_cmd_callback(const std_msgs::msg::String::SharedPtr msg);
    void person_position_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    
    // TF相关
    bool check_tf_availability();
    
    // 串口配置
    std::string serial_port_name_;
    int baud_rate_;
    boost::asio::io_context io_context_;
    boost::asio::serial_port serial_port_;
    std::thread io_thread_;
    
    // 数据处理
    SerialProtocol protocol_;
    std::array<uint8_t, 1> read_buffer_;
    PersonAggregator person_aggregator_;
    
    // TF相关
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr tf_startup_timer_;
    bool tf_available_;
    int tf_check_count_;
    rclcpp::Time last_tf_warn_time_;
    
    // ROS发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr off_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr height_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr temp_pos_pub_;
    
    // ROS订阅者
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr temperature_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr duoji_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr person_pos_sub_;
};

} // namespace com_pkg
