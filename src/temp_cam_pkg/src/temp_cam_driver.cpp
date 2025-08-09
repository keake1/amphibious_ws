#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <std_msgs/msg/float32.hpp> // 用于发布温度数据

class ThermalCameraNode : public rclcpp::Node
{
public:
    ThermalCameraNode(const std::string &port_name)
        : Node("thermal_camera_node"),
          port_name_(port_name),
          io_context_(),
          serial_port_(io_context_)
    {
        // 创建温度数据发布器
        temperature_publisher_ = this->create_publisher<std_msgs::msg::Float32>("temperature", 10);
        
        // 初始化节点
        initialize_node();
    }

    ~ThermalCameraNode()
    {
        cleanup();
    }

private:
    void initialize_node()
    {
        try
        {
            // 打开串口
            serial_port_.open(port_name_);
            serial_port_.set_option(boost::asio::serial_port_base::baud_rate(460800));
            serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
            serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
            
            // 启动异步接收
            start_async_receive();

            // 启动IO线程
            io_thread_ = std::thread([this]()
                                     { io_context_.run(); });

            // 发送初始化命令
            send_command({0x5A, 0x01, 0x00});
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            send_command({0x5A, 0x01, 0x03});
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            send_command({0x5A, 0x19, 0x03});
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            RCLCPP_INFO(this->get_logger(), "ThermalCameraNode initialized and activated.");
        }
        catch (const boost::system::system_error &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize thermal camera node: %s", e.what());
            throw;
        }
    }

    void cleanup()
    {
        if (serial_port_.is_open())
        {
            send_command({0x5A, 0x01, 0x00}); // 关闭命令
            serial_port_.close();
        }
        io_context_.stop();
        if (io_thread_.joinable())
        {
            io_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(), "ThermalCameraNode cleaned up.");
    }

    // 发送指令
    void send_command(const std::vector<uint8_t> &command)
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        boost::asio::async_write(serial_port_, boost::asio::buffer(command),
                                 [this](boost::system::error_code ec, std::size_t /*bytes_transferred*/)
                                 {
                                     if (ec)
                                     {
                                         RCLCPP_ERROR(this->get_logger(), "Failed to send command: %s", ec.message().c_str());
                                     }
                                     else
                                     {
                                         RCLCPP_INFO(this->get_logger(), "Command sent successfully");
                                     }
                                 });
    }

    void parse_data(const std::array<uint8_t, 256> &buffer, std::size_t bytes_transferred)
    {
        if (bytes_transferred != 16)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid data length: %zu bytes (expected 16 bytes)", bytes_transferred);
            return;
        }

        // 提取第 4 和第 5 个字节组成的温度数据
        uint16_t raw_temperature = (buffer[3] << 8) | buffer[4]; // 第 4 和第 5 个字节
        float temperature = static_cast<float>(raw_temperature) / 100.0f - 40.0f; // 转换为真实温度

        // 发布温度数据
        auto message = std_msgs::msg::Float32();
        message.data = temperature;
        temperature_publisher_->publish(message);

        //RCLCPP_INFO(this->get_logger(), "Temperature: %.2f °C", temperature);
    }

    void start_async_receive()
    {
        serial_port_.async_read_some(boost::asio::buffer(receive_buffer_),
                                     [this](boost::system::error_code ec, std::size_t bytes_transferred)
                                     {
                                         if (!ec)
                                         {
                                             std::lock_guard<std::mutex> lock(data_mutex_);
                                             parse_data(receive_buffer_, bytes_transferred);
                                             start_async_receive();
                                         }
                                         else
                                         {
                                             RCLCPP_ERROR(this->get_logger(), "Receive error: %s", ec.message().c_str());
                                         }
                                     });
    }

    std::string port_name_;
    boost::asio::io_context io_context_;
    boost::asio::serial_port serial_port_;
    std::thread io_thread_;
    std::mutex write_mutex_;
    std::mutex data_mutex_;
    std::array<uint8_t, 256> receive_buffer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperature_publisher_; // 温度数据发布器
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    try
    {
        auto node = std::make_shared<ThermalCameraNode>("/dev/thermal_cam"); // 替换为实际的串口名称
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("thermal_camera"), "Exception in main: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}