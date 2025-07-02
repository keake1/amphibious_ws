#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <boost/asio.hpp>
#include <vector>
#include <cmath>

using namespace boost::asio;
using namespace std::chrono_literals;

class SerialComNode : public rclcpp::Node
{
public:
    // 构造函数，初始化节点名称、串口配置和定时器
    SerialComNode() : Node("serial_com_node_without_imu"), serial_port_name_("/dev/ttyS1"), baud_rate_(921600), serial_port_(io_context_)
    {
        initialize_serial_port();      // 初始化串口
        initialize_ros_components();  // 初始化ROS相关组件
        initialize_timers();          // 初始化定时器
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
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock()); // 创建TF缓冲区
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); // 创建TF监听器
        
        // 新增: 创建 Twist 话题订阅者
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/tracked_pose", 10, 
            std::bind(&SerialComNode::twist_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "订阅话题: /tracked_pose");
    }

    // 初始化定时器，用于定时发送TF数据
    void initialize_timers()
    {
        tf_timer_ = this->create_wall_timer(10ms, [this]() { send_tf_data(); }); // 每10ms调用send_tf_data
    }

    // 发送TF数据
    void send_tf_data()
    {
        try {
            // 获取从 "odom" 到 "base_link" 的变换
            auto transform = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
            send_position_data(transform);    // 发送位置信息
            send_orientation_data(transform); // 发送方向信息
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what()); // 打印TF错误信息
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

    // 发送速度信息
    void send_velocity_data(const geometry_msgs::msg::Twist& twist)
    {
        // 将浮点速度转换为 int16_t (厘米/秒)
        int16_t vx = static_cast<int16_t>(twist.linear.x * 100);
        int16_t vy = static_cast<int16_t>(twist.linear.y * 100);
        
        std::vector<uint8_t> payload;
        append_to_data(payload, vx); // 添加 x 方向速度
        append_to_data(payload, vy); // 添加 y 方向速度
        send_serial_data(0x51, payload); // 发送数据，类型为 0x10
        
        RCLCPP_DEBUG(this->get_logger(), "发送速度数据: vx=%d cm/s, vy=%d cm/s", vx, vy);
    }

    // 成员变量
    std::string serial_port_name_; // 串口名称
    int baud_rate_;                // 波特率
    io_context io_context_;        // Boost IO上下文
    serial_port serial_port_;      // 串口对象

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_; // TF缓冲区
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // TF监听器

    rclcpp::TimerBase::SharedPtr tf_timer_; // 定时器
    
    // 新增: Twist 消息订阅者
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv); // 初始化ROS
    rclcpp::spin(std::make_shared<SerialComNode>()); // 运行节点
    rclcpp::shutdown(); // 关闭ROS
    return 0;
}