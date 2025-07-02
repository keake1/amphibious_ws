#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <array>
#include <memory>
#include <vector>
#include <cmath>
#include <deque>
#include <algorithm> // For std::clamp

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

class VelocityPIDNode : public rclcpp::Node {
public:
    VelocityPIDNode() : Node("velocity_pid_node") {
        // 声明参数，移除了 integral_limit_factor
        this->declare_parameter("kp", 30.0);
        this->declare_parameter("ki", 0.3);
        this->declare_parameter("kd", 0.5);
        this->declare_parameter("pwm_limit", 100.0);
        this->declare_parameter("deadzone", 0.01);
        this->declare_parameter("startup_pwm", 20.0);

        // 读取参数
        double kp = this->get_parameter("kp").as_double();
        double ki = this->get_parameter("ki").as_double();
        double kd = this->get_parameter("kd").as_double();
        double pwm_limit = this->get_parameter("pwm_limit").as_double();
        double deadzone = this->get_parameter("deadzone").as_double();
        startup_pwm_ = this->get_parameter("startup_pwm").as_double();

        // 创建PID控制器，构造函数已简化
        for (int i = 0; i < 4; ++i) {
            pid_vec_.emplace_back(std::make_unique<PIDController>(
                kp, ki, kd, -pwm_limit, pwm_limit, deadzone));
        }

        // 订阅实际轮速（由velocity_by_encoder节点发布）
        wheel_vel_measured_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/wheel_velocities_measured", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() == 4) {
                    for (int i = 0; i < 4; ++i) {
                        actual_wheel_vel_[i] = msg->data[i];
                    }
                }
            });

        // 订阅目标轮速
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

        // 发布PWM
        pwm_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_pwms", 10);

        // 定时器 - 50Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&VelocityPIDNode::control_loop, this));

        last_time_ = this->now();
        has_target_ = false;

        RCLCPP_INFO(this->get_logger(), 
            "优化版增量式速度PID节点启动 - kp:%.1f, ki:%.2f, kd:%.2f, PWM限制:%.1f", 
            kp, ki, kd, pwm_limit);
    }

private:
    int near_zero_count_ = 0;

    void control_loop() {
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
        
        pwm_pub_->publish(pwm_msg);

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
        std_msgs::msg::Float32MultiArray pwm_msg;
        pwm_msg.data.resize(4);
        std::fill(pwm_msg.data.begin(), pwm_msg.data.end(), 0.0);
        pwm_pub_->publish(pwm_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_vel_measured_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pwm_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::unique_ptr<PIDController>> pid_vec_;
    std::array<double, 4> target_wheel_vel_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> actual_wheel_vel_ = {0.0, 0.0, 0.0, 0.0};
    double startup_pwm_;
    bool has_target_;
    int debug_counter_ = 0;
    rclcpp::Time last_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelocityPIDNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}