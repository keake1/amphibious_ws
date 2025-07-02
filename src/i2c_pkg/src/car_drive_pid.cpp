#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

// 设置I2C总线号
#define I2C_BUS "/dev/i2c-0"

// 设置四路电机驱动模块的I2C地址
#define MOTOR_ADDR 0x34

// 寄存器地址
#define ADC_BAT_ADDR 0x00
#define MOTOR_TYPE_ADDR 0x14
#define MOTOR_ENCODER_POLARITY_ADDR 0x15
#define MOTOR_FIXED_PWM_ADDR 0x1F
#define MOTOR_FIXED_SPEED_ADDR 0x33
#define MOTOR_ENCODER_TOTAL_ADDR 0x3C

// 电机类型具体值
#define MOTOR_TYPE_WITHOUT_ENCODER 0
#define MOTOR_TYPE_TT 1
#define MOTOR_TYPE_N20 2
#define MOTOR_TYPE_JGB37_520_12V_110RPM 3 // 默认

class CarDrivePIDNode : public rclcpp::Node
{
public:
    CarDrivePIDNode() : Node("car_drive_pid_node"), i2c_fd_(-1)
    {
        // 声明参数
        this->declare_parameter("speed_scale", 1.0);  // 速度缩放因子
        this->declare_parameter("speed_limit", 50);   // 速度限制（绝对值）
        
        // 读取参数
        speed_scale_ = this->get_parameter("speed_scale").as_double();
        speed_limit_ = this->get_parameter("speed_limit").as_int();
        
        // 初始化I2C设备
        if (!init_i2c()) {
            RCLCPP_ERROR(this->get_logger(), "I2C初始化失败，退出节点");
            rclcpp::shutdown();
            return;
        }

        // 初始化电机
        motor_init();

        // 创建状态发布定时器，每秒读取并发布一次状态
        // status_timer_ = this->create_wall_timer(1s, std::bind(&CarDrivePIDNode::publish_status, this));

        // 初始化轮速数组
        wheel_speeds_ = {0, 0, 0, 0};
        
        // 设置初始状态 - 所有电机停止
        set_motor_speed(wheel_speeds_);

        // 订阅PID控制器发布的轮速
        wheel_velocity_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "wheel_velocities", 10, 
            std::bind(&CarDrivePIDNode::wheel_velocity_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "车辆PID驱动节点已初始化");
    }

    ~CarDrivePIDNode()
    {
        if (i2c_fd_ >= 0) {
            // 停止电机
            std::vector<int8_t> stop_speeds = {0, 0, 0, 0};
            set_motor_speed(stop_speeds);
            close(i2c_fd_);
            RCLCPP_INFO(this->get_logger(), "I2C设备已关闭");
        }
    }

private:
    int i2c_fd_;  // I2C文件描述符
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_velocity_sub_;
    
    // 电机速度
    std::vector<int8_t> wheel_speeds_;
    
    // 参数
    double speed_scale_;  // 速度缩放因子
    int speed_limit_;     // 速度限制

    // 初始化I2C
    bool init_i2c()
    {
        i2c_fd_ = open(I2C_BUS, O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "无法打开I2C设备: %s", I2C_BUS);
            return false;
        }

        if (ioctl(i2c_fd_, I2C_SLAVE, MOTOR_ADDR) < 0) {
            RCLCPP_ERROR(this->get_logger(), "无法设置I2C从设备地址");
            close(i2c_fd_);
            i2c_fd_ = -1;
            return false;
        }

        return true;
    }

    // 电机初始化
    void motor_init()
    {
        // 设置电机类型
        i2c_write_byte(MOTOR_TYPE_ADDR, MOTOR_TYPE_JGB37_520_12V_110RPM);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 设置编码极性
        i2c_write_byte(MOTOR_ENCODER_POLARITY_ADDR, 0);
        
        RCLCPP_INFO(this->get_logger(), "电机已初始化");
    }

    // 写入单个字节
    bool i2c_write_byte(uint8_t reg, uint8_t value)
    {
        uint8_t buf[2] = {reg, value};
        if (write(i2c_fd_, buf, 2) != 2) {
            RCLCPP_ERROR(this->get_logger(), "I2C写入失败");
            return false;
        }
        return true;
    }

    // 写入块数据
    bool i2c_write_block(uint8_t reg, const std::vector<int8_t>& values)
    {
        std::vector<uint8_t> buf(values.size() + 1);
        buf[0] = reg;
        std::memcpy(buf.data() + 1, values.data(), values.size());
        
        if (write(i2c_fd_, buf.data(), buf.size()) != static_cast<ssize_t>(buf.size())) {
            RCLCPP_ERROR(this->get_logger(), "I2C块写入失败");
            return false;
        }
        return true;
    }

    // 读取块数据
    bool i2c_read_block(uint8_t reg, uint8_t* values, size_t length)
    {
        if (write(i2c_fd_, &reg, 1) != 1) {
            RCLCPP_ERROR(this->get_logger(), "I2C写入寄存器地址失败");
            return false;
        }
        
        if (read(i2c_fd_, values, length) != static_cast<ssize_t>(length)) {
            RCLCPP_ERROR(this->get_logger(), "I2C读取数据失败");
            return false;
        }
        return true;
    }

    // 设置电机速度
    void set_motor_speed(const std::vector<int8_t>& speed)
    {
        if (i2c_write_block(MOTOR_FIXED_SPEED_ADDR, speed)) {
            // RCLCPP_INFO(this->get_logger(), "设置电机速度: [%d, %d, %d, %d]", 
            //         speed[0], speed[1], speed[2], speed[3]);
        }
    }

    // 轮速回调函数
    void wheel_velocity_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 4) {
            RCLCPP_ERROR(this->get_logger(), "接收到的轮速数据不正确，期望4个值，实际%zu个", msg->data.size());
            return;
        }
        
        // 将浮点数速度转换为整数
        for (size_t i = 0; i < 4; ++i) {
            // 应用速度缩放因子并转换为int8_t
            int scaled_speed = static_cast<int>(msg->data[i] * speed_scale_);
            if (i == 3 or i == 4) scaled_speed = -scaled_speed; // 反转第二和第三个轮子的速度
            // 限制速度在有效范围内
            if (scaled_speed > speed_limit_) {
                scaled_speed = speed_limit_;
            } else if (scaled_speed < -speed_limit_) {
                scaled_speed = -speed_limit_;
            }
            
            wheel_speeds_[i] = static_cast<int8_t>(scaled_speed);
        }
        
        // 发送速度到电机控制器
        set_motor_speed(wheel_speeds_);
    }

    // 读取并发布状态
    void publish_status()
    {
        // 读取电池电压
        uint8_t battery[2];
        if (i2c_read_block(ADC_BAT_ADDR, battery, 2)) {
            uint16_t voltage = battery[0] | (battery[1] << 8);
            RCLCPP_INFO(this->get_logger(), "电池电压 = %d mV", voltage);
        }
        
        // 读取编码器值
        uint8_t encoder_data[16];
        if (i2c_read_block(MOTOR_ENCODER_TOTAL_ADDR, encoder_data, 16)) {
            int32_t encoder_values[4];
            for (int i = 0; i < 4; i++) {
                encoder_values[i] = encoder_data[i*4] | 
                                   (encoder_data[i*4+1] << 8) | 
                                   (encoder_data[i*4+2] << 16) | 
                                   (encoder_data[i*4+3] << 24);
            }
            
            RCLCPP_INFO(this->get_logger(), "编码器值: Encode1 = %d, Encode2 = %d, Encode3 = %d, Encode4 = %d",
                    encoder_values[0], encoder_values[1], encoder_values[2], encoder_values[3]);
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CarDrivePIDNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}