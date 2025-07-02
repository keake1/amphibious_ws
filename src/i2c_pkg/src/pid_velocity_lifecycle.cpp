#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <array>
#include <memory>
#include <vector>
#include <cmath>
#include <deque>
#include <algorithm> // For std::clamp

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// 优化的增量式PID控制器类
class PIDController {
public:
    // 构造函数已简化，移除了不再需要的 integral_limit
    PIDController(double kp, double ki, double kd, double min_output, double max_output, double deadzone = 0.0)
        : kp_(kp), ki_(ki), kd_(kd), 
          min_output_(min_output), max_output_(max_output),
          deadzone_(deadzone),
          prev_error_(0.0), prev_prev_error_(0.0), last_output_(0.0), second_run_(true), n(0) {}

    void reset() {
        prev_error_ = 0.0;
        prev_prev_error_ = 0.0;
        last_output_ = 0.0;
        second_run_ = true;
        n = 0;
    }

    double compute(double error, double dt) {
        if (std::abs(error) < deadzone_) {
            error = 0.0;
        }

        double delta_output;
        if (second_run_ || dt <= 1e-6) { // 增加对dt过小的判断
            if (n < 2)
            {
                n++;
            }
            else
            {
                second_run_ = false;
            }
            delta_output = 0.0;
            
        } else {
            // 增量式PID核心公式: Δu(k) = Kp*(e(k)-e(k-1)) + Ki*e(k)*dt + Kd*(e(k)-2e(k-1)+e(k-2))/dt
            double p_term = kp_ * (error - prev_error_);
            double i_term = ki_ * error * dt;
            double d_term = kd_ * (error - 2 * prev_error_ + prev_prev_error_) / dt;
            delta_output = p_term + i_term + d_term;
        }

        double new_output = last_output_ + delta_output;

        // 使用 std::clamp 简化输出限制 (C++17)
        new_output = std::clamp(new_output, min_output_, max_output_);

        // 更新状态
        prev_prev_error_ = prev_error_;
        prev_error_ = error;
        last_output_ = new_output;

        return new_output;
    }

private:
    double kp_, ki_, kd_;
    double min_output_, max_output_;
    double deadzone_;

    double prev_error_;
    double prev_prev_error_;
    double last_output_;
    bool second_run_;
    uint8_t n;
};

class VelocityPIDLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
    VelocityPIDLifecycleNode() 
        : LifecycleNode("velocity_pid_lifecycle_node"), 
          last_time_(this->get_clock()->now()) {
        
        // 声明参数 - 只在构造函数中声明，不创建资源
        this->declare_parameter("kp", 30.0);
        this->declare_parameter("ki", 0.3);
        this->declare_parameter("kd", 0.5);
        this->declare_parameter("pwm_limit", 100.0);
        this->declare_parameter("deadzone", 0.01);
        this->declare_parameter("startup_pwm", 20.0);
        this->declare_parameter("wheel_base_length", 0.21);  // 前后轮距
        this->declare_parameter("wheel_base_width", 0.21);   // 左右轮距
        
        RCLCPP_INFO(this->get_logger(), "小车速度PID生命周期节点已创建，等待配置");
    }

    // 配置节点阶段：创建资源但不启动
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "正在配置小车速度PID生命周期节点...");
        try {
            // 读取参数
            double kp = this->get_parameter("kp").as_double();
            double ki = this->get_parameter("ki").as_double();
            double kd = this->get_parameter("kd").as_double();
            double pwm_limit = this->get_parameter("pwm_limit").as_double();
            double deadzone = this->get_parameter("deadzone").as_double();
            startup_pwm_ = this->get_parameter("startup_pwm").as_double();
            wheel_base_length_ = this->get_parameter("wheel_base_length").as_double();
            wheel_base_width_ = this->get_parameter("wheel_base_width").as_double();
            
            RCLCPP_INFO(this->get_logger(), 
                "麦克纳姆轮底盘尺寸: 长度=%.2fm, 宽度=%.2fm", 
                wheel_base_length_, wheel_base_width_);
            
            // 创建PID控制器
            for (int i = 0; i < 4; ++i) {
                pid_vec_.emplace_back(std::make_unique<PIDController>(
                    kp, ki, kd, -pwm_limit, pwm_limit, deadzone));
            }
            
            // 创建生命周期发布者
            pwm_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
                "/wheel_pwms", 10);
            
            // 初始化状态变量
            has_target_ = false;
            std::fill(target_wheel_vel_.begin(), target_wheel_vel_.end(), 0.0);
            debug_counter_ = 0;
            
            RCLCPP_INFO(this->get_logger(), 
                "小车速度PID生命周期节点配置完成 - kp:%.1f, ki:%.2f, kd:%.2f, PWM限制:%.1f", 
                kp, ki, kd, pwm_limit);
            
            return CallbackReturn::SUCCESS;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "配置失败: %s", e.what());
            return CallbackReturn::ERROR;
        }
    }

    // 激活节点阶段：启动通信和处理
    CallbackReturn on_activate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "正在激活小车速度PID生命周期节点...");
        
        // 激活发布者
        pwm_pub_->on_activate();
        
        // 只订阅目标轮速和编码器轮速，不再订阅/tracked_pose
        wheel_vel_measured_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/wheel_velocities_measured", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() == 4) {
                    for (int i = 0; i < 4; ++i) {
                        actual_wheel_vel_[i] = msg->data[i];
                    }
                }
            });

        wheel_vel_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/wheel_velocities", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() == 4) {
                    for (int i = 0; i < 4; ++i) {
                        target_wheel_vel_[i] = msg->data[i];
                    }
                    has_target_ = true;
                }
            });
            
        // 创建定时器（只在激活状态执行控制循环）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&VelocityPIDLifecycleNode::control_loop, this));
        
        // 重置时间戳和PID控制器
        last_time_ = this->now();
        for (auto& pid : pid_vec_) {
            pid->reset();
        }
        
        RCLCPP_INFO(this->get_logger(), "小车速度PID生命周期节点已激活");
        return CallbackReturn::SUCCESS;
    }

    // 停用节点阶段：停止处理但保留资源
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "正在停用小车速度PID生命周期节点...");
        
        // 停止定时器
        timer_.reset();
        
        // 停止订阅者
        wheel_vel_sub_.reset();
        
        // 停用发布者
        pwm_pub_->on_deactivate();
        
        // 发送一次零PWM值确保安全停止
        publish_zero_pwm();
        
        // 重置状态
        has_target_ = false;
        
        RCLCPP_INFO(this->get_logger(), "小车速度PID生命周期节点已停用");
        return CallbackReturn::SUCCESS;
    }

    // 清理节点阶段：释放资源
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "正在清理小车速度PID生命周期节点资源...");
        
        // 清理PID控制器
        pid_vec_.clear();
        
        // 清理发布者
        pwm_pub_.reset();
        
        RCLCPP_INFO(this->get_logger(), "小车速度PID生命周期节点资源已清理");
        return CallbackReturn::SUCCESS;
    }

    // 关闭节点阶段：在任何状态下紧急关闭
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "正在关闭小车速度PID生命周期节点...");
        
        // 停止定时器（如果存在）
        timer_.reset();
        
        // 停止订阅者（如果存在）
        wheel_vel_sub_.reset();
        
        // 尝试发送零PWM以确保安全停止
        if (pwm_pub_ && pwm_pub_->is_activated()) {
            publish_zero_pwm();
        }
        
        // 清理所有资源
        pid_vec_.clear();
        pwm_pub_.reset();
        
        RCLCPP_INFO(this->get_logger(), "小车速度PID生命周期节点已关闭");
        return CallbackReturn::SUCCESS;
    }

private:
    void control_loop() {
        // 检查节点状态
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            return;
        }
        
        if (!has_target_) {
            publish_zero_pwm();
            return;
        }

        // 直接使用编码器反馈的实际轮速
        const std::array<double, 4>& actual_wheel_vel = actual_wheel_vel_;

        // 计算dt
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 1e-6 || dt > 0.1) dt = 0.02;
        last_time_ = now;

        // 速度环PID，输出PWM
        std_msgs::msg::Float32MultiArray pwm_msg;
        pwm_msg.data.resize(4);
        
        for (int i = 0; i < 4; ++i) {
            double error = target_wheel_vel_[i] - actual_wheel_vel[i];
            double pwm = pid_vec_[i]->compute(error, dt);
            if (std::abs(pwm) > 5.0 && std::abs(pwm) < startup_pwm_) {
                pwm = (pwm > 0) ? startup_pwm_ : -startup_pwm_;
            }
            pwm_msg.data[i] = pwm;
        }
        
        // 检查发布者是否激活
        if (pwm_pub_->is_activated()) {
            pwm_pub_->publish(pwm_msg);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "尝试发布PWM，但发布者未激活");
        }

        // 调试信息
        debug_counter_++;
        if (debug_counter_ % 25 == 0) {
            RCLCPP_DEBUG(this->get_logger(), 
                "目标轮速: [%.2f, %.2f, %.2f, %.2f], 实际轮速: [%.2f, %.2f, %.2f, %.2f]",
                target_wheel_vel_[0], target_wheel_vel_[1], target_wheel_vel_[2], target_wheel_vel_[3],
                actual_wheel_vel[0], actual_wheel_vel[1], actual_wheel_vel[2], actual_wheel_vel[3]);
        }
    }

    void publish_zero_pwm() {
        // 确保发布者处于激活状态
        if (pwm_pub_ && pwm_pub_->is_activated()) {
            std_msgs::msg::Float32MultiArray pwm_msg;
            pwm_msg.data.resize(4);
            std::fill(pwm_msg.data.begin(), pwm_msg.data.end(), 0.0);
            pwm_pub_->publish(pwm_msg);
        }
    }

    // 成员变量
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_vel_measured_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_vel_sub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32MultiArray>> pwm_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::unique_ptr<PIDController>> pid_vec_;
    std::array<double, 4> target_wheel_vel_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> actual_wheel_vel_ = {0.0, 0.0, 0.0, 0.0};
    double startup_pwm_;
    bool has_target_;
    int debug_counter_ = 0;
    rclcpp::Time last_time_;
    double wheel_base_length_;  // 前后轮距
    double wheel_base_width_;   // 左右轮距
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 创建生命周期节点
    auto node = std::make_shared<VelocityPIDLifecycleNode>();
    
    // 使用多线程执行器，以便同时处理生命周期回调和用户回调
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    
    // 启动执行器
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}