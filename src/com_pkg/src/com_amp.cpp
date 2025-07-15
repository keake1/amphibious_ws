#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <boost/asio.hpp>
#include <vector>
#include <cmath>
#include <array>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>

using namespace boost::asio;
using namespace std::chrono_literals;

// 定义数据包解析状态
enum class ParseState {
    WAIT_HEADER_1,
    WAIT_HEADER_2,
    WAIT_TYPE,
    WAIT_LENGTH,
    WAIT_DATA,
    WAIT_CHECKSUM,
    WAIT_ADD_CHECKSUM
};

class SerialComNode : public rclcpp::Node
{
public:
    // 构造函数，初始化节点名称、串口配置和定时器
    SerialComNode() : Node("com_amp"), 
                      serial_port_name_("/dev/ttyS1"), 
                      baud_rate_(921600), 
                      serial_port_(io_context_),
                      parse_state_(ParseState::WAIT_HEADER_1)
    {
        initialize_serial_port();      // 初始化串口
        initialize_ros_components();   // 初始化ROS相关组件
        
        // 延迟启动TF查找，给系统时间来初始化TF frames
        tf_startup_timer_ = this->create_wall_timer(
            std::chrono::seconds(3),  // 延迟3秒
            [this]() {
                tf_startup_timer_->cancel();  // 取消启动定时器
                initialize_timers();          // 启动TF定时器
                RCLCPP_INFO(this->get_logger(), "开始TF数据发送");
            });
        
        // 启动异步读取
        start_async_read();
        
        // 在单独的线程中运行IO上下文
        io_thread_ = std::thread([this]() {
            io_context_.run();
        });
        
    }
    
    ~SerialComNode() {
        // 停止IO上下文和线程
        io_context_.stop();
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
    }

private:
    // 初始化串口
    void initialize_serial_port()
    {
        try {
            serial_port_.open(serial_port_name_); // 打开串口
            serial_port_.set_option(serial_port_base::baud_rate(baud_rate_)); // 设置波特率
            serial_port_.set_option(serial_port_base::character_size(8));    // 设置数据位
            serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::none)); // 设置无校验位
            serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one)); // 设置停止位
            RCLCPP_INFO(this->get_logger(), "Serial port initialized"); // 打印初始化成功信息
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Port open failed: %s", e.what()); // 打印错误信息
            rclcpp::shutdown(); // 关闭ROS节点
        }
    }

    // 初始化ROS相关组件
    void initialize_ros_components()
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 创建 Twist 话题订阅者
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/tracked_pose", 10, 
            std::bind(&SerialComNode::twist_callback, this, std::placeholders::_1));

        // 创建模式切换消息订阅者
        mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/lifecycle_switch_cmd", 10, 
            std::bind(&SerialComNode::mode_switch_callback, this, std::placeholders::_1));
        
        // 创建模式切换消息发布者
        off_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/is_off", 10);

        // 创建高度数据发布者
        height_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/vehicle_height", 10);
        
        RCLCPP_INFO(this->get_logger(), "订阅话题: /tracked_pose");
        RCLCPP_INFO(this->get_logger(), "发布话题: /mode_switch");
        RCLCPP_INFO(this->get_logger(), "发布话题: /vehicle_height");
        
    }

    // 初始化定时器，用于定时发送TF数据
    void initialize_timers()
    {
        // 添加一个初始延迟，等待TF系统初始化
        tf_timer_ = this->create_wall_timer(10ms, [this]() { send_tf_data(); }); // 改为100ms，减少频率
        
        // 添加TF可用性检查标志
        tf_available_ = false;
        tf_check_count_ = 0;
        last_tf_warn_time_ = this->get_clock()->now();
    }

    // 开始异步读取串口数据
    void start_async_read()
    {
        serial_port_.async_read_some(
            boost::asio::buffer(&read_buffer_, 1),
            std::bind(&SerialComNode::handle_read, this,
                      std::placeholders::_1,
                      std::placeholders::_2));
    }

    // 处理读取到的数据
    void handle_read(const boost::system::error_code& error, std::size_t bytes_transferred)
    {
        if (!error) {
            // 处理接收到的字节
            for (std::size_t i = 0; i < bytes_transferred; ++i) {
                process_byte(read_buffer_[i]);
            }
            
            // 继续异步读取
            start_async_read();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Read error: %s", error.message().c_str());
            // 尝试重新开始读取
            start_async_read();
        }
    }

    // 处理每一个接收到的字节
    void process_byte(uint8_t byte)
    {
        switch (parse_state_) {
            case ParseState::WAIT_HEADER_1:
                if (byte == 0xAA) {
                    parse_state_ = ParseState::WAIT_HEADER_2;
                    frame_checksum_ = byte;
                    frame_add_checksum_ = byte;
                }
                break;
                
            case ParseState::WAIT_HEADER_2:
                if (byte == 0xFF) {
                    parse_state_ = ParseState::WAIT_TYPE;
                    frame_checksum_ += byte;
                    frame_add_checksum_ += frame_checksum_;
                } else {
                    parse_state_ = ParseState::WAIT_HEADER_1;
                }
                break;
                
            case ParseState::WAIT_TYPE:
                frame_type_ = byte;
                frame_checksum_ += byte;
                frame_add_checksum_ += frame_checksum_;
                parse_state_ = ParseState::WAIT_LENGTH;
                break;
                
            case ParseState::WAIT_LENGTH:
                frame_length_ = byte;
                frame_checksum_ += byte;
                frame_add_checksum_ += frame_checksum_;
                frame_data_.clear();
                
                if (frame_length_ > 0) {
                    parse_state_ = ParseState::WAIT_DATA;
                } else {
                    parse_state_ = ParseState::WAIT_CHECKSUM;
                }
                break;
                
            case ParseState::WAIT_DATA:
                frame_data_.push_back(byte);
                frame_checksum_ += byte;
                frame_add_checksum_ += frame_checksum_;
                
                if (frame_data_.size() >= frame_length_) {
                    parse_state_ = ParseState::WAIT_CHECKSUM;
                }
                break;
                
            case ParseState::WAIT_CHECKSUM:
                if (byte == frame_checksum_) {
                    parse_state_ = ParseState::WAIT_ADD_CHECKSUM;
                } else {
                    RCLCPP_WARN(this->get_logger(), "校验和错误: 期望 %02X, 接收 %02X", frame_checksum_, byte);
                    parse_state_ = ParseState::WAIT_HEADER_1;
                }
                break;
                
            case ParseState::WAIT_ADD_CHECKSUM:
                if (byte == frame_add_checksum_) {
                    // 数据包接收完成，解析数据
                    process_frame(frame_type_, frame_data_);
                } else {
                    RCLCPP_WARN(this->get_logger(), "累加校验和错误: 期望 %02X, 接收 %02X", frame_add_checksum_, byte);
                }
                parse_state_ = ParseState::WAIT_HEADER_1;
                break;
        }
    }

    // 处理完整数据帧
    void process_frame(uint8_t type, const std::vector<uint8_t>& data)
    {
        RCLCPP_DEBUG(this->get_logger(), "接收到数据帧: 类型=0x%02X, 长度=%zu", type, data.size());
        
        switch (type) {
            case 0x00:  // 模式切换指令
                if (data.size() >= 1) {
                    process_mode_command(data[0]);
                }
                break;
        
            case 0x01:  // 高度数据
                if (data.size() >= sizeof(int)) {
                    process_height_data(data);
                } else {
                    RCLCPP_WARN(this->get_logger(), "高度数据长度不足: %zu", data.size());
                }
                break;
                
            default:
                RCLCPP_INFO(this->get_logger(), "未知数据类型: 0x%02X", type);
                break;
        }
    }

    // 添加处理模式切换命令的方法
    void process_mode_command(uint8_t mode_command)
    {
        auto message = std::make_unique<std_msgs::msg::String>();
        
        switch (mode_command) {
                
            case 0x01:
                message->data = "fly_off";
                RCLCPP_INFO(this->get_logger(), "收到消息:飞机已经降落");
                break;
                
            default:
                RCLCPP_WARN(this->get_logger(), "收到未知模式切换指令: 0x%02X", mode_command);
                return;  // 不发布消息
        }
        
        // 发布模式切换消息
        off_pub_->publish(*message);
    }

    // 处理高度数据
    void process_height_data(const std::vector<uint8_t>& data)
    {
        // 打印每个字节的十六进制值
        // std::stringstream byte_stream;
        // byte_stream << "高度数据原始字节: ";
        // for (size_t i = 0; i < data.size(); ++i) {
        //     byte_stream << std::hex << std::uppercase << std::setw(2) << std::setfill('0') 
        //               << static_cast<int>(data[i]) << " ";
        // }
        // RCLCPP_INFO(this->get_logger(), "%s", byte_stream.str().c_str());
        
        // 假设高度数据为 int 类型，直接从数据中提取
        int height = extract_from_data<int>(data, 0);
        
        // 同时打印十进制值以便对比
        // RCLCPP_INFO(this->get_logger(), "解析后的高度值: %d (0x%08X)", height, height);
        
        // 发布高度数据
        std_msgs::msg::Int32 height_msg;
        height_msg.data = height;
        height_pub_->publish(height_msg);
    }

    // 从数据中提取指定类型的值
    template <typename T>
    T extract_from_data(const std::vector<uint8_t>& data, size_t offset)
    {
        if (offset + sizeof(T) > data.size()) {
            RCLCPP_ERROR(this->get_logger(), "数据解析越界: 偏移量=%zu, 大小=%zu, 数据长度=%zu", 
                       offset, sizeof(T), data.size());
            return T();
        }
        
        T value;
        std::memcpy(&value, data.data() + offset, sizeof(T));
        return value;
    }

    // 发送TF数据
    void send_tf_data()
    {
        try {
            // 首先检查TF frame是否存在
            if (!tf_available_) {
                // 检查 odom 和 base_link frame 是否存在
                if (tf_buffer_->canTransform("odom", "base_link", tf2::TimePointZero)) {
                    tf_available_ = true;
                    RCLCPP_INFO(this->get_logger(), "TF frames 'odom' 和 'base_link' 现在可用");
                } else {
                    tf_check_count_++;
                    // 只在前几次检查失败时打印信息，然后每30秒打印一次
                    auto current_time = this->get_clock()->now();
                    if (tf_check_count_ <= 5 || 
                        (current_time - last_tf_warn_time_).seconds() > 30.0) {
                        RCLCPP_WARN(this->get_logger(), 
                                   "等待TF frames 'odom' 和 'base_link' 可用... (检查次数: %d)", 
                                   tf_check_count_);
                        last_tf_warn_time_ = current_time;
                    }
                    return;
                }
            }
            
            // 获取从 "odom" 到 "base_link" 的变换
            auto transform = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
            send_position_data(transform);    // 发送位置信息
            send_orientation_data(transform); // 发送方向信息
            
        } catch (const tf2::TransformException& ex) {
            // 如果之前认为TF可用但现在失败了，重置状态
            if (tf_available_) {
                tf_available_ = false;
                tf_check_count_ = 0;
                RCLCPP_WARN(this->get_logger(), "TF变换丢失，重新等待: %s", ex.what());
            }
            
            // 限制错误日志频率
            auto current_time = this->get_clock()->now();
            if ((current_time - last_tf_warn_time_).seconds() > 5.0) {
                RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
                last_tf_warn_time_ = current_time;
            }
        }
    }

    // 将任意类型的数据追加到字节数组中
    template <typename T>
    void append_to_data(std::vector<uint8_t>& data, T value)
    {
        auto bytes = reinterpret_cast<uint8_t*>(&value); // 将数据转换为字节
        for (size_t i = 0; i < sizeof(T); ++i) {
            data.push_back(bytes[i]); // 逐字节追加到数组
        }
    }

    // 计算校验和
    std::pair<uint8_t, uint8_t> calculate_checksums(const std::vector<uint8_t>& data)
    {
        uint8_t checksum = 0, add_checksum = 0;
        for (auto byte : data) {
            checksum += byte;         // 累加字节值
            add_checksum += checksum; // 累加校验和
        }
        return {checksum, add_checksum}; // 返回校验和和累加校验和
    }

    // 发送串口数据
    void send_serial_data(uint8_t type, const std::vector<uint8_t>& payload)
    {
        std::vector<uint8_t> data;
        data.push_back(0xAA); // 起始字节
        data.push_back(0xFF); // 标识字节
        data.push_back(type); // 数据类型
        data.push_back(static_cast<uint8_t>(payload.size())); // 数据长度
        data.insert(data.end(), payload.begin(), payload.end()); // 插入数据负载

        auto [cksm, add_cksm] = calculate_checksums(data); // 计算校验和
        data.push_back(cksm);      // 添加校验和
        data.push_back(add_cksm);  // 添加累加校验和

        boost::asio::write(serial_port_, boost::asio::buffer(data)); // 通过串口发送数据
    }

    // 发送位置信息
    void send_position_data(const geometry_msgs::msg::TransformStamped& transform)
    {
        float x = transform.transform.translation.x * 100; // 转换为厘米
        float y = transform.transform.translation.y * 100; // 转换为厘米

        std::vector<uint8_t> payload;
        append_to_data(payload, x); // 添加x坐标
        append_to_data(payload, y); // 添加y坐标
        send_serial_data(0x08, payload); // 发送数据
    }

    // 发送方向信息
    void send_orientation_data(const geometry_msgs::msg::TransformStamped& transform)
    {
        tf2::Quaternion q;
        tf2::fromMsg(transform.transform.rotation, q); // 从消息中提取四元数
        
        // 使用矩阵方法获取欧拉角
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw); // 获取roll, pitch, yaw角度
        
        float yaw_float = static_cast<float>(yaw); // 转换为float类型
        
        std::vector<uint8_t> payload;
        append_to_data(payload, yaw_float); // 添加偏航角
        send_serial_data(0x09, payload); // 发送数据
    }

    // Twist 消息回调函数
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        send_velocity_data(*msg);
    }

    // 模式切换消息回调函数
    void mode_switch_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "收到模式切换消息: %s", msg->data.c_str());
        if (msg->data == "fly_mode_on") send_serial_data(0x00, std::vector<uint8_t>{0x01});
    }

    // 发送速度信息
    void send_velocity_data(const geometry_msgs::msg::Twist& twist)
    {
        // 将浮点速度转换为 int16_t (厘米/秒)
        int16_t vx = static_cast<int16_t>(twist.linear.x * 100);
        int16_t vy = static_cast<int16_t>(twist.linear.y * 100);
        
        std::vector<uint8_t> payload;
        append_to_data(payload, vx); // 添加 x 方向速度
        append_to_data(payload, vy); // 添加 y 方向速度
        send_serial_data(0x51, payload); // 发送数据，类型为 0x51
        
        RCLCPP_DEBUG(this->get_logger(), "发送速度数据: vx=%d cm/s, vy=%d cm/s", vx, vy);
    }

    // 成员变量
    std::string serial_port_name_; // 串口名称
    int baud_rate_;                // 波特率
    io_context io_context_;        // Boost IO上下文
    serial_port serial_port_;      // 串口对象
    std::thread io_thread_;        // IO线程

    // 数据解析相关变量
    std::array<uint8_t, 1> read_buffer_;  // 单字节读取缓冲区
    ParseState parse_state_;              // 解析状态
    uint8_t frame_type_;                  // 数据帧类型
    uint8_t frame_length_;                // 数据帧长度
    std::vector<uint8_t> frame_data_;     // 数据帧负载
    uint8_t frame_checksum_;              // 校验和
    uint8_t frame_add_checksum_;          // 累加校验和

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_; // TF缓冲区
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // TF监听器

    rclcpp::TimerBase::SharedPtr tf_timer_; // 定时器
    rclcpp::TimerBase::SharedPtr tf_startup_timer_; // 启动延迟定时器
    
    // TF状态跟踪变量
    bool tf_available_;
    int tf_check_count_;
    rclcpp::Time last_tf_warn_time_;
    
    // Twist 消息订阅者
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

    // 模式切换消息订阅者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;

    // 模式切换消息发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr off_pub_;

    // 高度数据发布者
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr height_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv); // 初始化ROS
    rclcpp::spin(std::make_shared<SerialComNode>()); // 运行节点
    rclcpp::shutdown(); // 关闭ROS
    return 0;
}
