#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <chrono>
#include <memory>
#include <cmath>
#include <deque>

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// 优化的PID控制器类
class PIDController {
public:
    PIDController(double kp, double ki, double kd, double min_output, double max_output,
                  double integral_limit = 0.0, double deadzone = 0.0, int derivative_filter_size = 3,
                  double integral_region = 0.0)  // 添加积分区域参数
        : kp_(kp), ki_(ki), kd_(kd), 
          min_output_(min_output), max_output_(max_output),
          integral_limit_(integral_limit > 0 ? integral_limit : max_output),
          deadzone_(deadzone), derivative_filter_size_(derivative_filter_size),
          integral_region_(integral_region),  // 初始化积分区域
          prev_error_(0.0), integral_(0.0), first_run_(true) {}
    
    // 重置PID状态
    void reset() {
        prev_error_ = 0.0;
        integral_ = 0.0;
        first_run_ = true;
        error_history_.clear();
    }
    
    // 计算PID输出
    double compute(double error, double dt) {
        // 死区处理
        if (std::abs(error) < deadzone_) {
            error = 0.0;
        }
        
        // 只有在误差小于积分区域时才累加积分项
        if (integral_region_ <= 0.0 || std::abs(error) < integral_region_) {
            // 积分项累加（带积分饱和）
            integral_ += error * dt;
            
            // 积分饱和限制
            if (integral_ > integral_limit_) {
                integral_ = integral_limit_;
            } else if (integral_ < -integral_limit_) {
                integral_ = -integral_limit_;
            }
        } else {
            // 当误差超出积分区域时，逐渐减小积分项而不是直接清零
            integral_ *= 0.95;  // 使用衰减因子
        }
        
        // 计算微分项（带滤波）
        double derivative = 0.0;
        if (!first_run_ && dt > 0.0) {
            derivative = (error - prev_error_) / dt;
            
            // 微分项滤波
            if (derivative_filter_size_ > 1) {
                error_history_.push_back(derivative);
                if (error_history_.size() > static_cast<size_t>(derivative_filter_size_)) {
                    error_history_.pop_front();
                }
                
                // 计算平均值作为滤波结果
                double sum = 0.0;
                for (double val : error_history_) {
                    sum += val;
                }
                derivative = sum / error_history_.size();
            }
        }
        first_run_ = false;
        
        // 计算PID输出
        double proportional = kp_ * error;
        double integral_term = ki_ * integral_;
        double derivative_term = kd_ * derivative;
        
        double output = proportional + integral_term + derivative_term;
        
        // 输出限制（抗饱和）
        if (output > max_output_) {
            output = max_output_;
            // 抗积分饱和：如果输出饱和且积分项还在增大，则停止积分累加
            if (ki_ * error * dt > 0 && integral_ > 0) {
                integral_ -= error * dt;
            } else if (ki_ * error * dt < 0 && integral_ < 0) {
                integral_ -= error * dt;
            }
        } else if (output < min_output_) {
            output = min_output_;
            // 抗积分饱和
            if (ki_ * error * dt > 0 && integral_ > 0) {
                integral_ -= error * dt;
            } else if (ki_ * error * dt < 0 && integral_ < 0) {
                integral_ -= error * dt;
            }
        }
        
        // 保存当前误差用于下次计算
        prev_error_ = error;
        
        return output;
    }
    
    // 动态调整PID参数
    void setPID(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }
    
    // 获取PID各项的值（用于调试）
    void getComponents(double& p, double& i, double& d) const {
        p = kp_ * prev_error_;
        i = ki_ * integral_;
        d = kd_ * (error_history_.empty() ? 0.0 : error_history_.back());
    }

private:
    double kp_, ki_, kd_;
    double min_output_, max_output_;
    double integral_limit_;     // 积分限制
    double deadzone_;          // 死区
    int derivative_filter_size_; // 微分滤波器大小
    double integral_region_;    // 积分区域，只有误差小于此值时才累加积分
    
    double prev_error_;
    double integral_;
    bool first_run_;
    std::deque<double> error_history_; // 用于微分项滤波
};

class CarPIDLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
    CarPIDLifecycleNode()
        : LifecycleNode("car_pid_lifecycle_node"), 
          last_time_(this->get_clock()->now()) {
        
        // 只在构造函数中声明参数，不创建资源
        this->declare_parameter("position_kp", 2.0);
        this->declare_parameter("position_ki", 0.1);
        this->declare_parameter("position_kd", 0.5);
        this->declare_parameter("angle_kp", 3.0);
        this->declare_parameter("angle_ki", 0.05);
        this->declare_parameter("angle_kd", 0.8);
        this->declare_parameter("max_linear_speed", 50.0);
        this->declare_parameter("max_angular_speed", 2.0);
        this->declare_parameter("position_deadzone", 0.02);  // 2cm死区
        this->declare_parameter("angle_deadzone", 0.05);     // 约3度死区
        this->declare_parameter("integral_limit_factor", 0.5); // 积分限制因子
        // 添加积分区域参数
        this->declare_parameter("position_integral_region", 0.3);  // 默认30cm积分区域
        this->declare_parameter("angle_integral_region", 0.5);     // 默认约28.6度积分区域
        
        RCLCPP_INFO(this->get_logger(), "小车PID生命周期节点已创建");
    }

    // 配置节点阶段：创建资源但不启动
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "正在配置小车PID生命周期节点...");
        
        try {
            // 读取参数
            double position_kp = this->get_parameter("position_kp").as_double();
            double position_ki = this->get_parameter("position_ki").as_double();
            double position_kd = this->get_parameter("position_kd").as_double();
            double angle_kp = this->get_parameter("angle_kp").as_double();
            double angle_ki = this->get_parameter("angle_ki").as_double();
            double angle_kd = this->get_parameter("angle_kd").as_double();
            max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
            max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
            double position_deadzone = this->get_parameter("position_deadzone").as_double();
            double angle_deadzone = this->get_parameter("angle_deadzone").as_double();
            double integral_limit_factor = this->get_parameter("integral_limit_factor").as_double();
            // 读取积分区域参数
            double position_integral_region = this->get_parameter("position_integral_region").as_double();
            double angle_integral_region = this->get_parameter("angle_integral_region").as_double();
            
            // 初始化目标位置
            target_x_ = 0.0;
            target_y_ = 0.0;
            target_yaw_ = 0.0;
            has_target_ = false;
            
            // 创建优化的PID控制器，添加积分区域参数
            position_pid_x_ = std::make_unique<PIDController>(
                position_kp, position_ki, position_kd, 
                -max_linear_speed_, max_linear_speed_,
                max_linear_speed_ * integral_limit_factor, position_deadzone, 5,
                position_integral_region);
                
            position_pid_y_ = std::make_unique<PIDController>(
                position_kp, position_ki, position_kd, 
                -max_linear_speed_, max_linear_speed_,
                max_linear_speed_ * integral_limit_factor, position_deadzone, 5,
                position_integral_region);
                
            angle_pid_ = std::make_unique<PIDController>(
                angle_kp, angle_ki, angle_kd, 
                -max_angular_speed_, max_angular_speed_,
                max_angular_speed_ * integral_limit_factor, angle_deadzone, 3,
                angle_integral_region);
            
            // 创建TF监听器
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            
            // 创建生命周期发布者
            wheel_velocity_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
                "wheel_velocities", 10);
            
            RCLCPP_INFO(this->get_logger(), "小车PID生命周期节点配置完成");
            RCLCPP_INFO(this->get_logger(), "位置积分区域: %.2f m, 角度积分区域: %.2f rad (%.2f°)", 
                       position_integral_region, angle_integral_region, 
                       angle_integral_region * 180.0 / M_PI);
            return CallbackReturn::SUCCESS;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "配置失败: %s", e.what());
            return CallbackReturn::ERROR;
        }
    }

    // 激活节点阶段：启动通信和处理
    CallbackReturn on_activate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "正在激活小车PID生命周期节点...");
        
        // 创建订阅者（只在激活状态接收消息）
        target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_position", 10, 
            std::bind(&CarPIDLifecycleNode::target_callback, this, std::placeholders::_1));
        
        // 创建定时器（只在激活状态执行控制循环）
        timer_ = this->create_wall_timer(
            10ms, std::bind(&CarPIDLifecycleNode::control_loop, this));
        
        // 激活发布者
        wheel_velocity_pub_->on_activate();
        
        RCLCPP_INFO(this->get_logger(), "小车PID生命周期节点已激活");
        return CallbackReturn::SUCCESS;
    }

    // 停用节点阶段：停止处理但保留资源
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "正在停用小车PID生命周期节点...");
        
        // 停止定时器
        timer_.reset();
        
        // 停止订阅者
        target_sub_.reset();
        
        // 停用发布者
        wheel_velocity_pub_->on_deactivate();
        
        // 重置PID控制器状态
        if (position_pid_x_) position_pid_x_->reset();
        if (position_pid_y_) position_pid_y_->reset();
        if (angle_pid_) angle_pid_->reset();
        
        RCLCPP_INFO(this->get_logger(), "小车PID生命周期节点已停用");
        return CallbackReturn::SUCCESS;
    }

    // 清理节点阶段：释放资源
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "正在清理小车PID生命周期节点资源...");
        
        // 清理资源
        wheel_velocity_pub_.reset();
        position_pid_x_.reset();
        position_pid_y_.reset();
        angle_pid_.reset();
        tf_listener_.reset();
        tf_buffer_.reset();
        
        RCLCPP_INFO(this->get_logger(), "小车PID生命周期节点资源已清理");
        return CallbackReturn::SUCCESS;
    }

    // 关闭节点阶段：在任何状态下紧急关闭
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(this->get_logger(), "正在关闭小车PID生命周期节点...");
        
        // 停止定时器（如果存在）
        timer_.reset();
        
        // 停止订阅者（如果存在）
        target_sub_.reset();
        
        // 清理所有资源（无论当前状态）
        wheel_velocity_pub_.reset();
        position_pid_x_.reset();
        position_pid_y_.reset();
        angle_pid_.reset();
        tf_listener_.reset();
        tf_buffer_.reset();
        
        RCLCPP_INFO(this->get_logger(), "小车PID生命周期节点已关闭");
        return CallbackReturn::SUCCESS;
    }

private:
    // 计算坐标变换中的yaw角
    double get_yaw_from_transform(const geometry_msgs::msg::TransformStamped& transform) {
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }
    
    // 目标位置回调
    void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 检查节点状态
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            RCLCPP_WARN(this->get_logger(), "收到目标位置，但节点未激活，忽略消息");
            return;
        }
        
        target_x_ = msg->pose.position.x;
        target_y_ = msg->pose.position.y;
        
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        target_yaw_ = yaw;
        
        has_target_ = true;
        
        // 重置PID控制器
        position_pid_x_->reset();
        position_pid_y_->reset();
        angle_pid_->reset();
        
        RCLCPP_INFO(this->get_logger(), "新目标: x=%.2f, y=%.2f, yaw=%.2f°", 
                   target_x_, target_y_, target_yaw_ * 180.0 / M_PI);
    }
    
    // 主控制循环
    void control_loop() {
        // 检查节点状态
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            return;
        }
        
        if (!has_target_) {
            wheel_velocities_ = {0.0, 0.0, 0.0, 0.0}; // 停止轮速
            publish_wheel_velocities();
            return;
        }
        
        try {
            auto transform = tf_buffer_->lookupTransform(
                "odom", "base_link", tf2::TimePointZero);
            
            double current_x = transform.transform.translation.x;
            double current_y = transform.transform.translation.y;
            double current_yaw = get_yaw_from_transform(transform);
            
            // 计算误差
            double error_x = target_x_ - current_x;
            double error_y = target_y_ - current_y;
            double error_yaw = target_yaw_ - current_yaw;
            
            // 角度规范化
            while (error_yaw > M_PI) error_yaw -= 2.0 * M_PI;
            while (error_yaw < -M_PI) error_yaw += 2.0 * M_PI;
            
            // 计算时间间隔
            auto now = this->get_clock()->now();
            double dt = (now - last_time_).seconds();
            if (dt <= 0.0 || dt > 0.1) dt = 0.01;  // 限制dt范围
            last_time_ = now;
            
            // PID计算
            double vx = position_pid_x_->compute(error_x, dt);
            double vy = position_pid_y_->compute(error_y, dt);
            double omega = angle_pid_->compute(error_yaw, dt);
            
            // 计算并发布轮速
            calculate_wheel_velocities(vx, vy, omega);
            publish_wheel_velocities();
            
            // 检查是否到达目标
            double distance_error = std::sqrt(error_x*error_x + error_y*error_y);
            if (distance_error < 0.05 && std::abs(error_yaw) < 0.1) {
                static int arrived_count = 0;
                arrived_count++;
                if (arrived_count > 50) {  // 持续0.5秒到达目标
                    RCLCPP_INFO(this->get_logger(), "已到达目标位置");
                    has_target_ = false;  // 停止控制
                    arrived_count = 0;
                    position_pid_x_->reset();
                    position_pid_y_->reset();
                    angle_pid_->reset();
                }
            }
            
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "TF异常: %s", ex.what());
        }
    }
    
    // 计算麦克纳姆轮速度
    void calculate_wheel_velocities(double vx, double vy, double omega) {
        // 车体尺寸参数（单位：米）
        double wheel_base = 0.20;  // 轮距（左右轮之间的距离）
        double wheel_track = 0.21; // 轴距（前后轮之间的距离）
        
        // 计算轮中心到车体中心的距离
        double L = wheel_track / 2.0;  // 前后距离的一半
        double W = wheel_base / 2.0;   // 左右距离的一半
        
        // 麦克纳姆轮运动学逆解
        double rotation_factor = (L + W) * omega;
        
        // 根据原代码中轮子的顺序调整（0: 右前, 1: 左前, 2: 左后, 3: 右后）
        wheel_velocities_[0] = vx - vy - rotation_factor;  // 右前轮（0）
        wheel_velocities_[1] = vx + vy + rotation_factor;  // 左前轮（1）
        wheel_velocities_[2] = vx + vy - rotation_factor;  // 左后轮（2）
        wheel_velocities_[3] = vx - vy + rotation_factor;  // 右后轮（3）
        
        // 轮速标准化（确保不超过最大速度限制）
        double max_wheel_speed = 0.0;
        for (int i = 0; i < 4; i++) {
            double abs_speed = std::abs(wheel_velocities_[i]);
            if (abs_speed > max_wheel_speed) {
                max_wheel_speed = abs_speed;
            }
        }
        
        if (max_wheel_speed > max_linear_speed_) {
            double scale = max_linear_speed_ / max_wheel_speed;
            for (int i = 0; i < 4; i++) {
                wheel_velocities_[i] *= scale;
            }
        }
    }
    
    // 发布轮速
    void publish_wheel_velocities() {
        // 检查发布者是否激活
        if (!wheel_velocity_pub_->is_activated()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                               "尝试发布轮速，但发布者未激活");
            return;
        }
        
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data.resize(4);
        for (int i = 0; i < 4; i++) {
            msg.data[i] = static_cast<float>(wheel_velocities_[i]);
        }
        wheel_velocity_pub_->publish(msg);
    }
    
    // 成员变量
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<PIDController> position_pid_x_;
    std::unique_ptr<PIDController> position_pid_y_;
    std::unique_ptr<PIDController> angle_pid_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32MultiArray>> wheel_velocity_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    
    double target_x_, target_y_, target_yaw_;
    double max_linear_speed_, max_angular_speed_;
    std::array<double, 4> wheel_velocities_ = {0.0, 0.0, 0.0, 0.0};
    rclcpp::Time last_time_;
    bool has_target_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 创建生命周期节点
    auto node = std::make_shared<CarPIDLifecycleNode>();
    
    // 使用多线程执行器，以便处理生命周期回调和用户回调
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    
    // 启动执行器
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
