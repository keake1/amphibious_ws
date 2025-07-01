#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <boost/asio.hpp>
#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace boost::asio;
using namespace std::chrono_literals;

class SerialComNode : public rclcpp::Node
{
public:
    SerialComNode() : Node("serial_com_node"), serial_port_name_("/dev/ttyS2"), baud_rate_(115200), serial_port_(io_context_)
    {
        initialize_serial_port();
        initialize_ros_components();
        initialize_timers();
        initialize_calibration();
    }

private:
    // 初始化函数集
    void initialize_serial_port()
    {
        try {
            serial_port_.open(serial_port_name_);
            serial_port_.set_option(serial_port_base::baud_rate(baud_rate_));
            serial_port_.set_option(serial_port_base::character_size(8));
            serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
            serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
            RCLCPP_INFO(this->get_logger(), "Serial port initialized");
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Port open failed: %s", e.what());
            rclcpp::shutdown();
        }
    }

    void initialize_ros_components()
    {
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("velocity", 10);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void initialize_timers()
    {
        receive_timer_ = this->create_wall_timer(10ms, [this]() { receive_imu_data(); });
        velocity_timer_ = this->create_wall_timer(10ms, [this]() { compute_and_send_velocity(); });
        tf_timer_ = this->create_wall_timer(10ms, [this]() { send_tf_data(); });
    }

    void initialize_calibration()
    {
        calibration_frame_count_ = 0;
        accel_bias_ = {0.0, 0.0, 0.0};
        calibrated_ = false;
        last_position_ = {0.0, 0.0};
        last_time_ = this->now();
    }

    // 核心功能函数
    void send_tf_data()
    {
        try {
            auto transform = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
            
            send_position_data(transform);
            send_orientation_data(transform);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
        }
    }

    void receive_imu_data()
    {
        try {
            std::vector<uint8_t> buffer(256);
            size_t bytes_read = serial_port_.read_some(boost::asio::buffer(buffer));
            
            if (bytes_read >= 4 && validate_header(buffer)) {
                process_imu_packet(buffer);
            }
        } catch (const boost::system::system_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
        }
    }

    void compute_and_send_velocity()
    {
        try {
            auto transform = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
            auto [vx, vy] = calculate_velocity(transform);
            publish_velocity(vx, vy);
            send_velocity_data(vx, vy);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
        }
    }

    // 工具函数
    template <typename T>
    void append_to_data(std::vector<uint8_t>& data, T value)
    {
        auto bytes = reinterpret_cast<uint8_t*>(&value);
        for (size_t i = 0; i < sizeof(T); ++i) {
            data.push_back(bytes[i]);
        }
    }

    std::pair<uint8_t, uint8_t> calculate_checksums(const std::vector<uint8_t>& data)
    {
        uint8_t checksum = 0, add_checksum = 0;
        for (auto byte : data) {
            checksum += byte;
            add_checksum += checksum;
        }
        return {checksum, add_checksum};
    }

    void send_serial_data(uint8_t type, const std::vector<uint8_t>& payload)
    {
        std::vector<uint8_t> data;
        data.clear();  // 确保数据清空
        data.push_back(0xAA);
        data.push_back(0xFF);
        data.push_back(type);
        data.push_back(static_cast<uint8_t>(payload.size()));
        data.insert(data.end(), payload.begin(), payload.end());

        auto [cksm, add_cksm] = calculate_checksums(data);
        data.push_back(cksm);
        data.push_back(add_cksm);

        boost::asio::write(serial_port_, boost::asio::buffer(data));
    }
    // 数据处理函数
    void send_position_data(const geometry_msgs::msg::TransformStamped& transform)
    {
        float x = transform.transform.translation.x * 100;
        float y = transform.transform.translation.y * 100;
        
        std::vector<uint8_t> payload;
        append_to_data(payload, x);
        append_to_data(payload, y);
        send_serial_data(0x08, payload);
    }

    void send_orientation_data(const geometry_msgs::msg::TransformStamped& transform)
    {
        tf2::Quaternion q;
        tf2::fromMsg(transform.transform.rotation, q);
        float yaw = tf2::impl::getYaw(q);
        
        std::vector<uint8_t> payload;
        append_to_data(payload, yaw);
        send_serial_data(0x09, payload);
    }

    struct Velocity {
        double vx;
        double vy;
    };
    
    Velocity calculate_velocity(const geometry_msgs::msg::TransformStamped& transform)
    {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
    
        if (dt <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "Time interval is zero or negative, skipping velocity calculation.");
            return {0.0, 0.0};
        }
    
        double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;
        double vx = (x - last_position_.first) / dt * 100;
        double vy = (y - last_position_.second) / dt * 100;
    
        last_position_ = {x, y};
        last_time_ = current_time;
    
        return {vx, vy};
    }

    void send_velocity_data(double vx, double vy)
    {
        std::vector<uint8_t> payload{0x01,0x01};  // Mode
        append_to_data<int16_t>(payload, static_cast<int16_t>(vx));
        append_to_data<int16_t>(payload, static_cast<int16_t>(vy));
        send_serial_data(0x10, payload);
    }

    // IMU 数据处理
    bool validate_header(const std::vector<uint8_t>& buffer)
    {
        return buffer[0] == 0xAA && buffer[1] == 0xFF && buffer[2] == 0x01;
    }

    void process_imu_packet(const std::vector<uint8_t>& buffer)
    {
        uint8_t data_length = buffer[3];
        if (buffer.size() < static_cast<size_t>(4 + data_length + 2)) return;

        auto [cksm, add_cksm] = calculate_checksums({buffer.begin(), buffer.begin() + 4 + data_length});
        if (cksm != buffer[4+data_length] || add_cksm != buffer[4+data_length+1]) return;

        process_imu_data(buffer);
    }

    void process_imu_data(const std::vector<uint8_t>& buffer)
    {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header = std_msgs::msg::Header().set__stamp(this->now()).set__frame_id("imu_link");

        process_acceleration(imu_msg, buffer);
        process_gyro(imu_msg, buffer);
        process_orientation(imu_msg, buffer);

        imu_publisher_->publish(imu_msg);
    }

    void process_acceleration(sensor_msgs::msg::Imu& msg, const std::vector<uint8_t>& buffer)
    {
        double ax = parse_value<int16_t>(buffer, 4) * 0.01;
        double ay = parse_value<int16_t>(buffer, 6) * 0.01;
        double az = parse_value<int16_t>(buffer, 8) * 0.01;

        if (!calibrated_) {
            handle_calibration(ax, ay, az);
            return;
        }

        msg.linear_acceleration.x = ax - accel_bias_.x;
        msg.linear_acceleration.y = ay - accel_bias_.y;
        msg.linear_acceleration.z = az - accel_bias_.z;
    }

    void handle_calibration(double ax, double ay, double az)
    {
        if (calibration_frame_count_++ < 300) {
            accel_bias_.x += ax;
            accel_bias_.y += ay;
            accel_bias_.z += az;
            return;
        }

        accel_bias_.x /= 300;
        accel_bias_.y /= 300;
        accel_bias_.z = accel_bias_.z / 300 - 9.81;
        calibrated_ = true;
        RCLCPP_INFO(this->get_logger(), "IMU calibration completed");
    }

    void process_gyro(sensor_msgs::msg::Imu& msg, const std::vector<uint8_t>& buffer)
    {
        msg.angular_velocity.x = parse_value<int16_t>(buffer, 10) / 16.384 * M_PI / 180.0;
        msg.angular_velocity.y = parse_value<int16_t>(buffer, 12) / 16.384 * M_PI / 180.0;
        msg.angular_velocity.z = parse_value<int16_t>(buffer, 14) / 16.384 * M_PI / 180.0;
    }

    void process_orientation(sensor_msgs::msg::Imu& msg, const std::vector<uint8_t>& buffer)
    {
        msg.orientation.w = parse_value<int16_t>(buffer, 16) * 0.0001;
        msg.orientation.x = parse_value<int16_t>(buffer, 18) * 0.0001;
        msg.orientation.y = parse_value<int16_t>(buffer, 20) * 0.0001;
        msg.orientation.z = parse_value<int16_t>(buffer, 22) * 0.0001;
    }

    template <typename T>
    T parse_value(const std::vector<uint8_t>& buffer, size_t offset)
    {
        T value;
        memcpy(&value, &buffer[offset], sizeof(T));
        return value;
    }

    void publish_velocity(double vx, double vy)
    {
        geometry_msgs::msg::Twist velocity_msg;
        velocity_msg.linear.x = vx;
        velocity_msg.linear.y = vy;
        velocity_publisher_->publish(velocity_msg);
        //RCLCPP_INFO(this->get_logger(), "Published velocity: vx=%.2f, vy=%.2f", vx, vy);
    }

    // 成员变量
    std::string serial_port_name_;
    int baud_rate_;
    io_context io_context_;
    serial_port serial_port_;
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::TimerBase::SharedPtr receive_timer_, velocity_timer_, tf_timer_;
    
    struct {
        double x = 0.0, y = 0.0, z = 0.0;
    } accel_bias_;
    
    std::pair<double, double> last_position_;
    rclcpp::Time last_time_;
    int calibration_frame_count_;
    bool calibrated_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialComNode>());
    rclcpp::shutdown();
    return 0;
}