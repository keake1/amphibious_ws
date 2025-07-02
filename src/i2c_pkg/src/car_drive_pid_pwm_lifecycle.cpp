#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <vector>
#include <cstring>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>

#define I2C_BUS "/dev/i2c-0"  // 如需用1请改为"/dev/i2c-1"
#define MOTOR_ADDR 0x34
#define MOTOR_FIXED_PWM_ADDR 0x1F
#define MOTOR_FIXED_SPEED_ADDR 0x33

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CarDrivePIDPWMLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
    CarDrivePIDPWMLifecycleNode() 
        : LifecycleNode("car_drive_pid_pwm_lifecycle_node"), 
          i2c_fd_(-1), 
          running_(false) {
        
        // 在构造函数中只声明参数，不初始化资源
        this->declare_parameter("i2c_bus", I2C_BUS);
        this->declare_parameter("motor_addr", MOTOR_ADDR);
        
        RCLCPP_INFO(this->get_logger(), "小车PWM驱动生命周期节点已创建，等待配置");
    }
    
    ~CarDrivePIDPWMLifecycleNode() {
        // 确保在析构时清理所有资源
        if (running_) {
            running_ = false;
            if (send_thread_.joinable()) {
                send_thread_.join();
            }
        }
        
        if (i2c_fd_ >= 0) {
            stop_motors();
            close(i2c_fd_);
            i2c_fd_ = -1;
        }
    }
    
    // 配置节点阶段：初始化资源，但不启动处理
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "正在配置小车PWM驱动生命周期节点...");
        
        try {
            // 获取参数
            std::string i2c_bus = this->get_parameter("i2c_bus").as_string();
            int motor_addr = this->get_parameter("motor_addr").as_int();
            
            // 打开I2C设备
            i2c_fd_ = open(i2c_bus.c_str(), O_RDWR);
            if (i2c_fd_ < 0) {
                RCLCPP_ERROR(this->get_logger(), "无法打开I2C设备: %s", i2c_bus.c_str());
                return CallbackReturn::FAILURE;
            }
            
            if (ioctl(i2c_fd_, I2C_SLAVE, motor_addr) < 0) {
                RCLCPP_ERROR(this->get_logger(), "无法设置I2C从设备地址: 0x%x", motor_addr);
                close(i2c_fd_);
                i2c_fd_ = -1;
                return CallbackReturn::FAILURE;
            }
            
            // 初始化last_pwm_
            last_pwm_ = {0, 0, 0, 0};
            
            // 创建发布者(可选，用于反馈状态)
            
            // 创建订阅者（在激活时才会被使用）
            pwm_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "/wheel_pwms", 10,
                std::bind(&CarDrivePIDPWMLifecycleNode::pwm_callback, this, std::placeholders::_1)
            );
            
            RCLCPP_INFO(this->get_logger(), "小车PWM驱动生命周期节点配置完成");
            return CallbackReturn::SUCCESS;
            
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "配置失败: %s", e.what());
            // 清理资源
            if (i2c_fd_ >= 0) {
                close(i2c_fd_);
                i2c_fd_ = -1;
            }
            return CallbackReturn::ERROR;
        }
    }
    
    // 激活节点阶段：启动处理
    CallbackReturn on_activate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "正在激活小车PWM驱动生命周期节点...");
        
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "I2C设备未打开，无法激活");
            return CallbackReturn::ERROR;
        }
        
        // 启动后台线程持续写I2C
        running_ = true;
        send_thread_ = std::thread([this]() {
            while (rclcpp::ok() && running_) {
                std::vector<int8_t> pwm_copy;
                {
                    std::lock_guard<std::mutex> lock(pwm_mutex_);
                    pwm_copy = last_pwm_;
                }
                write_pwm(pwm_copy);
                // std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50Hz
            }
        });
        
        RCLCPP_INFO(this->get_logger(), "小车PWM驱动生命周期节点已激活");
        return CallbackReturn::SUCCESS;
    }
    
    // 停用节点阶段：停止处理，但保留资源
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "正在停用小车PWM驱动生命周期节点...");
        
        // 停止线程
        running_ = false;
        if (send_thread_.joinable()) {
            send_thread_.join();
        }
        
        // 停止电机
        stop_motors();
        
        RCLCPP_INFO(this->get_logger(), "小车PWM驱动生命周期节点已停用");
        return CallbackReturn::SUCCESS;
    }
    
    // 清理节点阶段：释放资源
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "正在清理小车PWM驱动生命周期节点资源...");
        
        // 确保线程已停止
        if (running_) {
            running_ = false;
            if (send_thread_.joinable()) {
                send_thread_.join();
            }
        }
        
        // 关闭I2C设备
        if (i2c_fd_ >= 0) {
            stop_motors();
            close(i2c_fd_);
            i2c_fd_ = -1;
        }
        
        // 清理订阅者
        pwm_sub_.reset();
        
        RCLCPP_INFO(this->get_logger(), "小车PWM驱动生命周期节点资源已清理");
        return CallbackReturn::SUCCESS;
    }
    
    // 关闭节点阶段：在任何状态下紧急关闭
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "正在关闭小车PWM驱动生命周期节点...");
        
        // 确保线程已停止
        if (running_) {
            running_ = false;
            if (send_thread_.joinable()) {
                send_thread_.join();
            }
        }
        
        // 关闭I2C设备
        if (i2c_fd_ >= 0) {
            stop_motors();
            close(i2c_fd_);
            i2c_fd_ = -1;
        }
        
        // 清理订阅者
        pwm_sub_.reset();
        
        RCLCPP_INFO(this->get_logger(), "小车PWM驱动生命周期节点已关闭");
        return CallbackReturn::SUCCESS;
    }

private:
    int i2c_fd_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pwm_sub_;
    std::vector<int8_t> last_pwm_;
    std::mutex pwm_mutex_;
    std::thread send_thread_;
    std::atomic<bool> running_;

    void pwm_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        // 检查节点状态
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                               "收到PWM消息，但节点未激活，忽略消息");
            return;
        }
        
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
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "尝试写PWM，但I2C设备未打开");
            return;
        }
        
        uint8_t buf[5];
        buf[0] = MOTOR_FIXED_PWM_ADDR;
        std::memcpy(buf + 1, pwm.data(), 4);
        ssize_t ret = write(i2c_fd_, buf, 5);
        if (ret != 5) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "I2C写入PWM失败: %zd", ret);
        }
    }

    void stop_motors() {
        if (i2c_fd_ < 0) {
            return;
        }
        
        std::vector<int8_t> pwm_zero = {0, 0, 0, 0};
        write_pwm(pwm_zero);
        usleep(100000); // 100ms
        RCLCPP_INFO(this->get_logger(), "所有电机已停止");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 创建生命周期节点
    auto node = std::make_shared<CarDrivePIDPWMLifecycleNode>();
    
    // 使用多线程执行器，以便处理生命周期回调和用户回调
    rclcpp::executors::MultiThreadedExecutor executor;
    
    executor.add_node(node->get_node_base_interface());
    
    // 启动执行器
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}