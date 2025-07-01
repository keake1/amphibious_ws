#include <rclcpp/rclcpp.hpp>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

// 设置I2C总线号，通常为1（在Python中是0，但C++通常使用/dev/i2c-1）
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

class CarDriveNode : public rclcpp::Node
{
public:
    CarDriveNode() : Node("car_drive_node"), i2c_fd_(-1)
    {
        // 初始化I2C设备
        if (!init_i2c()) {
            RCLCPP_ERROR(this->get_logger(), "I2C初始化失败，退出节点");
            rclcpp::shutdown();
            return;
        }

        // 初始化电机
        motor_init();

        // 创建定时器，每3秒切换电机方向
        timer_ = this->create_wall_timer(3s, std::bind(&CarDriveNode::timer_callback, this));

        // 创建状态发布定时器，每秒读取并发布一次状态
        status_timer_ = this->create_wall_timer(1s, std::bind(&CarDriveNode::publish_status, this));

        // 初始化电机速度
        speed1_ = {50, 0, 0, 0};
        speed2_ = {-50, 0, 0, 0};
        speed3_ = {0, 0, 0, 0};

        // 初始化PWM值（未使用但保留供参考）
        pwm1_ = {50, 50, 50, 50};
        pwm2_ = {-100, -100, -100, -100};
        pwm3_ = {0, 0, 0, 0};

        // 设置初始状态
        current_speed_ = &speed1_;
        
        // 首次设置电机速度
        set_motor_speed(*current_speed_);

        RCLCPP_INFO(this->get_logger(), "车辆驱动节点已初始化");
    }

    ~CarDriveNode()
    {
        if (i2c_fd_ >= 0) {
            // 停止电机
            set_motor_speed(speed3_);
            close(i2c_fd_);
            RCLCPP_INFO(this->get_logger(), "I2C设备已关闭");
        }
    }

private:
    int i2c_fd_;  // I2C文件描述符
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // 速度设置
    std::vector<int8_t> speed1_;
    std::vector<int8_t> speed2_;
    std::vector<int8_t> speed3_;
    
    // PWM设置（备用）
    std::vector<int8_t> pwm1_;
    std::vector<int8_t> pwm2_;
    std::vector<int8_t> pwm3_;
    
    // 当前速度指针
    std::vector<int8_t>* current_speed_;

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
            RCLCPP_INFO(this->get_logger(), "设置电机速度: [%d, %d, %d, %d]", 
                    speed[0], speed[1], speed[2], speed[3]);
        }
    }

    // 定时器回调，切换电机方向
    void timer_callback()
    {
        if (current_speed_ == &speed1_) {
            current_speed_ = &speed2_;
        } else {
            current_speed_ = &speed1_;
        }
        
        set_motor_speed(*current_speed_);
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
    auto node = std::make_shared<CarDriveNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}