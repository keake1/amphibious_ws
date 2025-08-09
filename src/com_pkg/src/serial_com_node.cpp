#include "com_pkg/serial_com_node.hpp"
#include <chrono>

using namespace boost::asio;
using namespace std::chrono_literals;

namespace com_pkg {

SerialComNode::SerialComNode() 
    : Node("com_amp")
    , serial_port_name_("/dev/ttyS1")
    , baud_rate_(921600)
    , serial_port_(io_context_)
    , tf_available_(false)
    , tf_check_count_(0) {
    
    initialize_serial_port();
    initialize_ros_components();
    
    // 延迟启动TF查找
    tf_startup_timer_ = this->create_wall_timer(
        std::chrono::seconds(3),
        [this]() {
            tf_startup_timer_->cancel();
            initialize_timers();
            RCLCPP_INFO(this->get_logger(), "开始TF数据发送");
        });
    
    start_async_read();
    
    // 在单独线程中运行IO上下文
    io_thread_ = std::thread([this]() {
        io_context_.run();
    });
}

SerialComNode::~SerialComNode() {
    io_context_.stop();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
}

void SerialComNode::initialize_serial_port() {
    try {
        serial_port_.open(serial_port_name_);
        serial_port_.set_option(serial_port_base::baud_rate(baud_rate_));
        serial_port_.set_option(serial_port_base::character_size(8));
        serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
        serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        RCLCPP_INFO(this->get_logger(), "串口初始化成功");
    } catch (const boost::system::system_error& e) {
        RCLCPP_ERROR(this->get_logger(), "串口打开失败: %s", e.what());
        rclcpp::shutdown();
    }
}

void SerialComNode::initialize_ros_components() {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    last_tf_warn_time_ = this->get_clock()->now();
    
    // 创建发布者
    off_pub_ = this->create_publisher<std_msgs::msg::String>("/is_off", 10);
    height_pub_ = this->create_publisher<std_msgs::msg::Int32>("/vehicle_height", 10);
    voltage_pub_ = this->create_publisher<std_msgs::msg::Float32>("/voltage", 10);
    temp_pos_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/temp_person_positions", 10);
    
    // 创建订阅者
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/tracked_pose", 10, 
        std::bind(&SerialComNode::twist_callback, this, std::placeholders::_1));
    
    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/lifecycle_switch_cmd", 10,
        std::bind(&SerialComNode::mode_switch_callback, this, std::placeholders::_1));
    
    temperature_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/temperature", 10,
        std::bind(&SerialComNode::temperature_callback, this, std::placeholders::_1));
    
    duoji_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/duoji_cmd", 10,
        std::bind(&SerialComNode::duoji_cmd_callback, this, std::placeholders::_1));
    
    person_pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/camera0/person_position", 10,
        std::bind(&SerialComNode::person_position_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "ROS组件初始化完成");
}

void SerialComNode::initialize_timers() {
    tf_timer_ = this->create_wall_timer(10ms, [this]() { send_tf_data(); });
}

void SerialComNode::start_async_read() {
    serial_port_.async_read_some(
        boost::asio::buffer(&read_buffer_, 1),
        std::bind(&SerialComNode::handle_read, this,
                  std::placeholders::_1,
                  std::placeholders::_2));
}

void SerialComNode::handle_read(const boost::system::error_code& error, std::size_t bytes_transferred) {
    if (!error) {
        for (std::size_t i = 0; i < bytes_transferred; ++i) {
            if (protocol_.process_byte(read_buffer_[i])) {
                process_received_frame();
            }
        }
        start_async_read();
    } else {
        RCLCPP_ERROR(this->get_logger(), "读取错误: %s", error.message().c_str());
        start_async_read();
    }
}

void SerialComNode::process_received_frame() {
    uint8_t type = protocol_.get_frame_type();
    const auto& data = protocol_.get_frame_data();
    
    RCLCPP_DEBUG(this->get_logger(), "接收到数据帧: 类型=0x%02X, 长度=%zu", type, data.size());
    
    switch (type) {
        case 0x00:
        case 0x04:
            if (!data.empty()) {
                process_mode_command(data[0]);
            }
            break;
        case 0x01:
            if (data.size() >= sizeof(int)) {
                process_height_data(data);
            } else {
                RCLCPP_WARN(this->get_logger(), "高度数据长度不足: %zu", data.size());
            }
            break;
        case 0x02:
            if (data.size() == 9) {
                process_temperature_position(data);
            } else {
                RCLCPP_WARN(this->get_logger(), "温度坐标数据长度错误: %zu", data.size());
            }
            break;
        case 0x03:
            if (data.size() == 2) {
                process_voltage_data(data);
            } else {
                RCLCPP_WARN(this->get_logger(), "电压数据长度错误: %zu", data.size());
            }
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "未知数据类型: 0x%02X", type);
            break;
    }
}

void SerialComNode::process_mode_command(uint8_t mode_command) {
    if (mode_command == 0x01) {
        auto message = std_msgs::msg::String();
        message.data = "fly_off";
        off_pub_->publish(message);
        RCLCPP_INFO(this->get_logger(), "收到消息: 飞机已经降落");
    } else {
        RCLCPP_WARN(this->get_logger(), "收到未知模式切换指令: 0x%02X", mode_command);
    }
}

void SerialComNode::process_height_data(const std::vector<uint8_t>& data) {
    int height = DataUtils::from_bytes<int>(data, 0);
    
    auto height_msg = std_msgs::msg::Int32();
    height_msg.data = height;
    height_pub_->publish(height_msg);
}

void SerialComNode::process_voltage_data(const std::vector<uint8_t>& data) {
    uint16_t voltage_raw = DataUtils::from_bytes<uint16_t>(data, 0);
    float voltage = static_cast<float>(voltage_raw) / 100.0f;
    
    auto voltage_msg = std_msgs::msg::Float32();
    voltage_msg.data = voltage;
    voltage_pub_->publish(voltage_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "发布电压数据: %.2f V", voltage);
}

void SerialComNode::process_temperature_position(const std::vector<uint8_t>& data) {
    float x = DataUtils::from_bytes<float>(data, 0);
    float y = DataUtils::from_bytes<float>(data, 4);
    uint8_t temp_flag = data[8];

    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.resize(3);
    msg.data[0] = x / 100.0f;
    msg.data[1] = y / 100.0f;
    msg.data[2] = static_cast<float>(temp_flag);

    temp_pos_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "发布温度坐标: x=%.2f, y=%.2f, 高温=%s",
                x, y, temp_flag == 0x01 ? "是" : "否");
}

void SerialComNode::send_serial_data(uint8_t type, const std::vector<uint8_t>& payload) {
    auto packet = SerialProtocol::create_packet(type, payload);
    boost::asio::write(serial_port_, boost::asio::buffer(packet));
}

bool SerialComNode::check_tf_availability() {
    if (!tf_available_) {
        if (tf_buffer_->canTransform("odom", "base_link", tf2::TimePointZero)) {
            tf_available_ = true;
            RCLCPP_INFO(this->get_logger(), "TF frames 'odom' 和 'base_link' 现在可用");
            return true;
        } else {
            tf_check_count_++;
            auto current_time = this->get_clock()->now();
            if (tf_check_count_ <= 5 || (current_time - last_tf_warn_time_).seconds() > 30.0) {
                RCLCPP_WARN(this->get_logger(), "等待TF frames可用... (检查次数: %d)", tf_check_count_);
                last_tf_warn_time_ = current_time;
            }
            return false;
        }
    }
    return true;
}

void SerialComNode::send_tf_data() {
    if (!check_tf_availability()) return;
    
    try {
        auto transform = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
        send_position_data(transform);
        send_orientation_data(transform);
    } catch (const tf2::TransformException& ex) {
        if (tf_available_) {
            tf_available_ = false;
            tf_check_count_ = 0;
            RCLCPP_WARN(this->get_logger(), "TF变换丢失，重新等待: %s", ex.what());
        }
        
        auto current_time = this->get_clock()->now();
        if ((current_time - last_tf_warn_time_).seconds() > 5.0) {
            RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
            last_tf_warn_time_ = current_time;
        }
    }
}

void SerialComNode::send_position_data(const geometry_msgs::msg::TransformStamped& transform) {
    float x = transform.transform.translation.x * 100; // 转换为厘米
    float y = transform.transform.translation.y * 100;

    std::vector<uint8_t> payload;
    DataUtils::append_to_vector(payload, x);
    DataUtils::append_to_vector(payload, y);
    send_serial_data(0x08, payload);
}

void SerialComNode::send_orientation_data(const geometry_msgs::msg::TransformStamped& transform) {
    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    float yaw_float = static_cast<float>(yaw);
    
    std::vector<uint8_t> payload;
    DataUtils::append_to_vector(payload, yaw_float);
    send_serial_data(0x09, payload);
}

void SerialComNode::send_velocity_data(const geometry_msgs::msg::Twist& twist) {
    int16_t vx = static_cast<int16_t>(twist.linear.x * 100);
    int16_t vy = static_cast<int16_t>(twist.linear.y * 100);
    
    std::vector<uint8_t> payload;
    DataUtils::append_to_vector(payload, vx);
    DataUtils::append_to_vector(payload, vy);
    send_serial_data(0x51, payload);
    
    RCLCPP_DEBUG(this->get_logger(), "发送速度数据: vx=%d cm/s, vy=%d cm/s", vx, vy);
}

void SerialComNode::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    send_velocity_data(*msg);
}

void SerialComNode::mode_switch_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "收到模式切换消息: %s", msg->data.c_str());
    if (msg->data == "fly_mode_on") {
        send_serial_data(0x00, std::vector<uint8_t>{0x01});
    }
}

void SerialComNode::temperature_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    int16_t temp_int = static_cast<int16_t>(msg->data);
    std::vector<uint8_t> payload;
    DataUtils::append_to_vector(payload, temp_int);
    send_serial_data(0x10, payload);
    RCLCPP_DEBUG(this->get_logger(), "发送温度数据: %.2f °C", msg->data);
}

void SerialComNode::duoji_cmd_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "收到舵机命令: %s", msg->data.c_str());
    if (msg->data == "duoji_on") {
        send_serial_data(0x12, std::vector<uint8_t>{0x01});
        RCLCPP_INFO(this->get_logger(), "发送舵机开启命令");
    }
}

void SerialComNode::person_position_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    if (person_aggregator_.add_coordinate(msg->x, msg->y)) {
        auto coordinates = person_aggregator_.get_all_coordinates();
        
        std::vector<uint8_t> payload;
        for (const auto& coord : coordinates) {
            int16_t x_int = static_cast<int16_t>(coord.first * 100);
            int16_t y_int = static_cast<int16_t>(coord.second * 100);
            DataUtils::append_to_vector(payload, x_int);
            DataUtils::append_to_vector(payload, y_int);
        }
        
        send_serial_data(0x11, payload);
        
        RCLCPP_INFO(this->get_logger(), "三人坐标已聚合并发送");
        person_aggregator_.reset_all();
    }
}

} // namespace com_pkg
