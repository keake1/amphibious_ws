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
        // 新增参数：速度来源选择
        this->declare_parameter("velocity_source", "tracked_pose"); // "encoder" 或 "tracked_pose"
        // 机器人几何参数（用于Twist转换为轮速）
        this->declare_parameter("wheel_base", 0.26);  // 轮距（前后轮距离，单位：米）
        this->declare_parameter("track_width", 0.24); // 轴距（左右轮距离，单位：米）
        this->declare_parameter("wheel_radius", 0.04); // 轮子半径（单位：米）

        // 读取参数
        double kp = this->get_parameter("kp").as_double();
        double ki = this->get_parameter("ki").as_double();
        double kd = this->get_parameter("kd").as_double();
        double pwm_limit = this->get_parameter("pwm_limit").as_double();
        double deadzone = this->get_parameter("deadzone").as_double();
	startup_pwm_ = this->get_parameter("startup_pwm").as_double();
        velocity_source_ = this->get_parameter("velocity_source").as_string();
        
        // 读取机器人几何参数
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        track_width_ = this->get_parameter("track_width").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();

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
                    std::lock_guard<std::mutex> lock(wheel_vel_mutex_);
                    for (int i = 0; i < 4; ++i) {
                        encoder_wheel_vel_[i] = msg->data[i];
                    }
                    has_encoder_vel_ = true;
                }
            });

        // 新增：订阅跟踪位姿速度
        tracked_pose_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/tracked_pose", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(wheel_vel_mutex_);
                twist_linear_x_ = msg->linear.x;
                twist_linear_y_ = msg->linear.y;
                twist_angular_z_ = msg->angular.z;
                
                // 将Twist消息转换为四个轮子的速度
                convertTwistToWheelVelocities();
                has_tracked_pose_ = true;
            });

        // 订阅目标轮速
        wheel_vel_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/wheel_velocities", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() == 4) {
                    std::lock_guard<std::mutex> lock(wheel_vel_mutex_);
                    for (int i = 0; i < 4; ++i) {
                        target_wheel_vel_[i] = msg->data[i];
                    }
                    has_target_ = true;
                }
            });

        // 发布PWM
        pwm_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_pwms", 10);

	timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&VelocityPIDNode::control_loop, this));

        last_time_ = this->now();
        has_target_ = false;
        has_encoder_vel_ = false;
        has_tracked_pose_ = false;

        RCLCPP_INFO(this->get_logger(), 
            "优化版增量式速度PID节点启动 - kp:%.1f, ki:%.2f, kd:%.2f, PWM限制:%.1f, 速度来源:%s", 
            kp, ki, kd, pwm_limit, velocity_source_.c_str());
    }

private:
    int near_zero_count_ = 0;

    // 将Twist消息转换为四个轮子的速度
    void convertTwistToWheelVelocities() {
        // 假设轮子排列：
        // 0: 左前, 1: 右前, 2: 左后, 3: 右后
        
        // 差速驱动运动学模型（适用于四轮差速或麦克纳姆轮）
        double half_wheel_base = wheel_base_ / 2.0;
        double half_track_width = track_width_ / 2.0;
        
        // 计算各轮子速度
        if (velocity_source_ == "tracked_pose") {
            // 四轮差速/麦克纳姆轮模型
            // 左前轮
            tracked_pose_wheel_vel_[1] = (twist_linear_x_ - twist_linear_y_ - 
                                        (half_wheel_base + half_track_width) * twist_angular_z_) / wheel_radius_;
            // 右前轮
            tracked_pose_wheel_vel_[0] = (twist_linear_x_ + twist_linear_y_ + 
                                        (half_wheel_base + half_track_width) * twist_angular_z_) / wheel_radius_;
            // 左后轮
            tracked_pose_wheel_vel_[2] = (twist_linear_x_ + twist_linear_y_ - 
                                        (half_wheel_base + half_track_width) * twist_angular_z_) / wheel_radius_;
            // 右后轮
            tracked_pose_wheel_vel_[3] = (twist_linear_x_ - twist_linear_y_ + 
                                        (half_wheel_base + half_track_width) * twist_angular_z_) / wheel_radius_;
                                        
            // 转换为线速度 (m/s)
            for (int i = 0; i < 4; ++i) {
                tracked_pose_wheel_vel_[i] *= wheel_radius_;
            }
        }
    }

    void control_loop() {
        std::lock_guard<std::mutex> lock(wheel_vel_mutex_);
        
        if (!has_target_) {
            publish_zero_pwm();
            return;
        }

        // 根据选择的来源决定使用哪个实际轮速
        std::array<double, 4> actual_wheel_vel;
        bool has_velocity = false;
        
        if (velocity_source_ == "encoder" && has_encoder_vel_) {
            actual_wheel_vel = encoder_wheel_vel_;
            has_velocity = true;
        } 
        else if (velocity_source_ == "tracked_pose" && has_tracked_pose_) {
            actual_wheel_vel = tracked_pose_wheel_vel_;
            has_velocity = true;
        }
        
        if (!has_velocity) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "未接收到有效的速度数据，来源：%s", velocity_source_.c_str());
            publish_zero_pwm();
            return;
        }

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
                "目标轮速: [%.2f, %.2f, %.2f, %.2f], 实际轮速: [%.2f, %.2f, %.2f, %.2f], 来源: %s",
                target_wheel_vel_[0], target_wheel_vel_[1], target_wheel_vel_[2], target_wheel_vel_[3],
                actual_wheel_vel[0], actual_wheel_vel[1], actual_wheel_vel[2], actual_wheel_vel[3],
                velocity_source_.c_str());
        }
    }

    void publish_zero_pwm() {
        std_msgs::msg::Float32MultiArray pwm_msg;
        pwm_msg.data.resize(4);
        std::fill(pwm_msg.data.begin(), pwm_msg.data.end(), 0.0);
        pwm_pub_->publish(pwm_msg);
    }

    // 订阅和发布
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_vel_measured_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr tracked_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pwm_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    // PID控制器
    std::vector<std::unique_ptr<PIDController>> pid_vec_;
    
    // 速度数据
    std::array<double, 4> target_wheel_vel_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> encoder_wheel_vel_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> tracked_pose_wheel_vel_ = {0.0, 0.0, 0.0, 0.0};
    
    // Twist数据
    double twist_linear_x_ = 0.0;
    double twist_linear_y_ = 0.0;
    double twist_angular_z_ = 0.0;
    
    // 机器人几何参数
    double wheel_base_;    // 轮距（前后轮距离）
    double track_width_;   // 轴距（左右轮距离）
    double wheel_radius_;  // 轮子半径
    
    // 控制参数
    double startup_pwm_;
    std::string velocity_source_;
    
    // 状态标志
    bool has_target_;
    bool has_encoder_vel_;
    bool has_tracked_pose_;
    
    // 其他
    int debug_counter_ = 0;
    rclcpp::Time last_time_;
    std::mutex wheel_vel_mutex_;  // 保护速度数据的互斥锁
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelocityPIDNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
