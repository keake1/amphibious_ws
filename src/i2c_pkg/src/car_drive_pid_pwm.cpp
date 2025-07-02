#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <vector>
#include <array>
#include <cstring>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;

// 寄存器地址常量
#define MOTOR_ADDR 0x34
#define ADC_BAT_ADDR 0x00
#define MOTOR_TYPE_ADDR 0x14
#define MOTOR_ENCODER_POLARITY_ADDR 0x15
#define MOTOR_FIXED_PWM_ADDR 0x1F
#define MOTOR_FIXED_SPEED_ADDR 0x33
#define MOTOR_ENCODER_TOTAL_ADDR 0x3C

// 电机类型
#define MOTOR_TYPE_JGB37_520_12V_110RPM 3

class CarDrivePIDPWMNode : public rclcpp::Node {
public:
    CarDrivePIDPWMNode() : Node("car_drive_pid_pwm_node"), i2c_fd_(-1), running_(true) {
        // 声明参数
        this->declare_parameter("i2c_bus", 0);  // I2C总线号

        // 获取参数
        int i2c_bus = this->get_parameter("i2c_bus").as_int();

        // 打开I2C
        char i2c_device[20];
        snprintf(i2c_device, sizeof(i2c_device), "/dev/i2c-%d", i2c_bus);

        i2c_fd_ = open(i2c_device, O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "无法打开I2C设备: %s", i2c_device);
            rclcpp::shutdown();
            return;
        }

        if (ioctl(i2c_fd_, I2C_SLAVE, MOTOR_ADDR) < 0) {
            RCLCPP_ERROR(this->get_logger(), "无法设置I2C从设备地址");
            close(i2c_fd_);
            rclcpp::shutdown();
            return;
        }

        // 初始化last_pwm_
        last_pwm_ = {0, 0, 0, 0};

        // 订阅PWM话题
        pwm_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/wheel_pwms", 10,
            std::bind(&CarDrivePIDPWMNode::pwm_callback, this, std::placeholders::_1)
        );

        // 启动后台线程持续写I2C
        send_thread_ = std::thread([this]() {
            while (rclcpp::ok() && running_) {
                std::vector<int8_t> pwm_copy;
                {
                    std::lock_guard<std::mutex> lock(pwm_mutex_);
                    pwm_copy = last_pwm_;
                }
                write_pwm(pwm_copy);
                //usleep(500); // 10ms
            }
        });

        RCLCPP_INFO(this->get_logger(), "PWM驱动节点已启动");
    }

    ~CarDrivePIDPWMNode() {
        running_ = false;
        if (send_thread_.joinable()) send_thread_.join();
        stop_motors();
        if (i2c_fd_ >= 0) {
            close(i2c_fd_);
        }
        RCLCPP_INFO(this->get_logger(), "I2C设备已关闭");
    }

private:
    int i2c_fd_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pwm_sub_;
    std::vector<int8_t> last_pwm_;
    std::mutex pwm_mutex_;
    std::thread send_thread_;
    std::atomic<bool> running_;

    void pwm_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() != 4) {
            RCLCPP_WARN(this->get_logger(), "收到的PWM数据长度不是4");
            return;
        }
        std::vector<int8_t> pwm(4);
        for (size_t i = 0; i < 4; ++i) {
            int val = static_cast<int>(std::round(msg->data[i]));
            if (i == 2 || i == 3) { // 后轮
                val = -val;
            }
            if (val > 100) val = 100;
            if (val < -100) val = -100;
            pwm[i] = static_cast<int8_t>(val);
        }
        {
            std::lock_guard<std::mutex> lock(pwm_mutex_);
            last_pwm_ = pwm;
        }
    }

    void write_pwm(const std::vector<int8_t>& pwm) {
        uint8_t buf[5];
        buf[0] = MOTOR_FIXED_PWM_ADDR;
        std::memcpy(buf + 1, pwm.data(), 4);
        ssize_t ret = write(i2c_fd_, buf, 5);
        if (ret != 5) {
            RCLCPP_ERROR(this->get_logger(), "I2C写入PWM失败");
        }
    }

    void stop_motors() {
        std::vector<int8_t> pwm_zero = {0, 0, 0, 0};
        write_pwm(pwm_zero);
        usleep(100000); // 100ms
        RCLCPP_INFO(this->get_logger(), "所有电机已停止");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CarDrivePIDPWMNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}