#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <cstring>
#include <vector>

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// I2C和电机驱动配置常量
constexpr int I2C_BUS = 0;  // I2C总线号
constexpr int MOTOR_ADDR = 0x34;  // 电机驱动的I2C地址

// 寄存器地址
constexpr uint8_t ADC_BAT_ADDR = 0x00;
constexpr uint8_t MOTOR_TYPE_ADDR = 0x14;
constexpr uint8_t MOTOR_ENCODER_POLARITY_ADDR = 0x15;
constexpr uint8_t MOTOR_FIXED_PWM_ADDR = 0x1F;
constexpr uint8_t MOTOR_FIXED_SPEED_ADDR = 0x33;
constexpr uint8_t MOTOR_ENCODER_TOTAL_ADDR = 0x3C;

// 电机类型
constexpr uint8_t MOTOR_TYPE_JGB37_520_12V_110RPM = 3;  // 默认电机类型

class EncoderVelocityLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    EncoderVelocityLifecycleNode()
    : rclcpp_lifecycle::LifecycleNode("encoder_velocity_lifecycle_node")
    {}

    ~EncoderVelocityLifecycleNode()
    {
        if (i2c_fd_ >= 0) {
            close(i2c_fd_);
        }
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        // 声明参数
        this->declare_parameter("encoder_counts_per_rev", 43000);  // 编码器每转计数
        this->declare_parameter("wheel_diameter", 0.08);  // 轮子直径，单位：米
        this->declare_parameter("i2c_bus", I2C_BUS);  // I2C总线号
        this->declare_parameter("publish_counts", false);  // 是否发布原始计数值

        // 获取参数
        encoder_counts_per_rev_ = this->get_parameter("encoder_counts_per_rev").as_int();
        wheel_diameter_ = this->get_parameter("wheel_diameter").as_double();
        int i2c_bus = this->get_parameter("i2c_bus").as_int();
        publish_counts_ = this->get_parameter("publish_counts").as_bool();

        // 计算轮子周长
        wheel_circumference_ = M_PI * wheel_diameter_;

        // 发布器
        velocity_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/wheel_velocities_measured", 10);

        if (publish_counts_) {
            encoder_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
                "/wheel_encoder_counts", 10);
        }

        // 初始化I2C
        char i2c_device[20];
        snprintf(i2c_device, sizeof(i2c_device), "/dev/i2c-%d", i2c_bus);

        i2c_fd_ = open(i2c_device, O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "无法打开I2C设备 %s", i2c_device);
            return CallbackReturn::FAILURE;
        }

        if (ioctl(i2c_fd_, I2C_SLAVE, MOTOR_ADDR) < 0) {
            RCLCPP_ERROR(this->get_logger(), "无法设置I2C从设备地址");
            close(i2c_fd_);
            i2c_fd_ = -1;
            return CallbackReturn::FAILURE;
        }

        // 初始化电机驱动
        init_motor_driver();

        // 记录初始编码器值
        if (!read_encoder_counts(prev_encoder_counts_)) {
            RCLCPP_ERROR(this->get_logger(), "无法读取初始编码器值");
            return CallbackReturn::FAILURE;
        }

        prev_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
                    "编码器速度生命周期节点已配置 - 每圈计数: %d, 轮直径: %.3fm, 周长: %.3fm",
                    encoder_counts_per_rev_, wheel_diameter_, wheel_circumference_);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "编码器速度生命周期节点激活");
        if (velocity_pub_) velocity_pub_->on_activate();
        if (encoder_pub_) encoder_pub_->on_activate();

        // 创建定时器，每10ms读取一次
        timer_ = this->create_wall_timer(
            10ms, std::bind(&EncoderVelocityLifecycleNode::timer_callback, this));
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "编码器速度生命周期节点停用");
        timer_.reset();
        if (velocity_pub_) velocity_pub_->on_deactivate();
        if (encoder_pub_) encoder_pub_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "编码器速度生命周期节点清理");
        timer_.reset();
        velocity_pub_.reset();
        encoder_pub_.reset();
        if (i2c_fd_ >= 0) {
            close(i2c_fd_);
            i2c_fd_ = -1;
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "编码器速度生命周期节点关闭");
        timer_.reset();
        velocity_pub_.reset();
        encoder_pub_.reset();
        if (i2c_fd_ >= 0) {
            close(i2c_fd_);
            i2c_fd_ = -1;
        }
        return CallbackReturn::SUCCESS;
    }

private:
    static constexpr int32_t ENCODER_ABS_THRESHOLD = 1000000; // 编码器绝对值阈值

    void init_motor_driver()
    {
        // 设置电机类型
        uint8_t data = MOTOR_TYPE_JGB37_520_12V_110RPM;
        if (i2c_write(MOTOR_TYPE_ADDR, &data, 1) < 0) {
            RCLCPP_ERROR(this->get_logger(), "设置电机类型失败");
            return;
        }

        // 给驱动器一些时间处理
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 设置编码器极性
        data = 0;
        if (i2c_write(MOTOR_ENCODER_POLARITY_ADDR, &data, 1) < 0) {
            RCLCPP_ERROR(this->get_logger(), "设置编码器极性失败");
            return;
        }

        // 重置编码器计数
        reset_encoder_counts();

        RCLCPP_INFO(this->get_logger(), "电机驱动初始化完成");
    }

    void reset_encoder_counts()
    {
        uint8_t reset_data[16] = {0};
        if (i2c_write(MOTOR_ENCODER_TOTAL_ADDR, reset_data, 16) < 0) {
            RCLCPP_ERROR(this->get_logger(), "重置编码器计数失败");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "编码器计数已重置");
    }

    bool read_encoder_counts(std::array<int32_t, 4>& counts)
    {
        uint8_t buffer[16];
        if (i2c_read(MOTOR_ENCODER_TOTAL_ADDR, buffer, 16) < 0) {
            RCLCPP_ERROR(this->get_logger(), "读取编码器计数失败");
            return false;
        }

        // 将字节数据转换为4个int32
        for (int i = 0; i < 4; i++) {
            counts[i] = buffer[i*4] | (buffer[i*4+1] << 8) |
                       (buffer[i*4+2] << 16) | (buffer[i*4+3] << 24);
        }

        return true;
    }

    int i2c_write(uint8_t reg, const uint8_t* data, size_t length)
    {
        std::vector<uint8_t> buffer(length + 1);
        buffer[0] = reg;
        memcpy(buffer.data() + 1, data, length);

        if (write(i2c_fd_, buffer.data(), length + 1) != static_cast<ssize_t>(length + 1)) {
            RCLCPP_ERROR(this->get_logger(), "I2C写入失败: %s", strerror(errno));
            return -1;
        }
        return 0;
    }

    int i2c_read(uint8_t reg, uint8_t* data, size_t length)
    {
        if (write(i2c_fd_, &reg, 1) != 1) {
            RCLCPP_ERROR(this->get_logger(), "I2C写寄存器地址失败: %s", strerror(errno));
            return -1;
        }

        if (read(i2c_fd_, data, length) != static_cast<ssize_t>(length)) {
            RCLCPP_ERROR(this->get_logger(), "I2C读取失败: %s", strerror(errno));
            return -1;
        }
        return 0;
    }

    void timer_callback()
    {
        // 读取当前编码器计数
        std::array<int32_t, 4> current_counts;
        if (!read_encoder_counts(current_counts)) {
            return;
        }

        // 获取当前时间
        auto current_time = this->now();
        double dt = (current_time - prev_time_).seconds();

        // 计算每个轮子的速度
        std::array<float, 4> wheel_velocities;
        std::array<int32_t, 4> count_diffs;

        for (int i = 0; i < 4; i++) {
            // 检查阈值，若超出则置0，并修正速度计算
            if (std::abs(current_counts[i]) >= ENCODER_ABS_THRESHOLD) {
                RCLCPP_WARN(this->get_logger(), "编码器%d绝对值超阈值，已置0", i);
                count_diffs[i] = current_counts[i] - prev_encoder_counts_[i];
                current_counts[i] = 0;
                // 立即写回硬件清零
                uint8_t reset_data[4] = {0, 0, 0, 0};
                uint8_t reg = MOTOR_ENCODER_TOTAL_ADDR + i * 4;
                i2c_write(reg, reset_data, 4);
            } else {
                count_diffs[i] = current_counts[i] - prev_encoder_counts_[i];
            }
            // 轮子0和1（前两个轮子）的编码器是反向计数的，需要取反
            if (i == 0 || i == 1) {
                count_diffs[i] = -count_diffs[i];
            }
        }

        for (int i = 0; i < 4; i++) {
            double rotations_per_second = static_cast<double>(count_diffs[i]) /
                                         encoder_counts_per_rev_ / dt;
            wheel_velocities[i] = rotations_per_second * wheel_circumference_;
        }

        // 更新前一次的数值
        prev_encoder_counts_ = current_counts;
        prev_time_ = current_time;

        // 发布轮速数据
        auto velocity_msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
        velocity_msg->data.resize(4);
        for (int i = 0; i < 4; i++) {
            velocity_msg->data[i] = wheel_velocities[i];
        }
        if (velocity_pub_ && velocity_pub_->is_activated()) {
            velocity_pub_->publish(std::move(velocity_msg));
        }

        // 可选：发布编码器原始计数
        if (publish_counts_ && encoder_pub_ && encoder_pub_->is_activated()) {
            auto counts_msg = std::make_unique<std_msgs::msg::Int32MultiArray>();
            counts_msg->data.resize(4);
            for (int i = 0; i < 4; i++) {
                counts_msg->data[i] = current_counts[i];
            }
            encoder_pub_->publish(std::move(counts_msg));
        }

        // 每隔10次（约1秒）打印一次日志
        static int log_counter = 0;
        if (++log_counter >= 10) {
            log_counter = 0;
            RCLCPP_INFO(this->get_logger(),
                "轮速(m/s): [%.2f, %.2f, %.2f, %.2f], 计数差: [%d, %d, %d, %d]",
                wheel_velocities[0], wheel_velocities[1],
                wheel_velocities[2], wheel_velocities[3],
                count_diffs[0], count_diffs[1], count_diffs[2], count_diffs[3]);
        }
    }

    // I2C文件描述符
    int i2c_fd_ = -1;

    // 参数
    int encoder_counts_per_rev_;
    double wheel_diameter_;
    double wheel_circumference_;
    bool publish_counts_;

    // 发布器
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32MultiArray>> velocity_pub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32MultiArray>> encoder_pub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 存储前一次的编码器值和时间
    std::array<int32_t, 4> prev_encoder_counts_ = {0, 0, 0, 0};
    rclcpp::Time prev_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EncoderVelocityLifecycleNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}