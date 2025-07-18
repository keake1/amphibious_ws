#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "amp_interfaces/msg/target_position.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <chrono>
#include <cmath>

class RescueTaskNode : public rclcpp::Node {
public:
    RescueTaskNode() : Node("rescue_task_test2"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        lifecycle_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/lifecycle_switch_cmd", 10);
        target_pub_ = this->create_publisher<amp_interfaces::msg::TargetPosition>("/target_position", 10);
        tf_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RescueTaskNode::tf_callback, this));
        step_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RescueTaskNode::step_callback, this));
        is_off_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/is_off", 10, std::bind(&RescueTaskNode::is_off_callback, this, std::placeholders::_1));
        current_step_ = 0;
        target_reached_time_ = 0.0;
        last_time_ = this->now();
        fly_off_received_ = false;
        RCLCPP_INFO(this->get_logger(), "RescueTaskNode started.");
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lifecycle_cmd_pub_;
    rclcpp::Publisher<amp_interfaces::msg::TargetPosition>::SharedPtr target_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr step_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr is_off_sub_;
    int current_step_;
    rclcpp::Time last_time_;
    double target_reached_time_;
    bool target1_reached_ = false;
    bool target2_reached_ = false;
    bool fly_off_received_ = false;
    geometry_msgs::msg::TransformStamped current_tf_;
    // TF可用性判断相关变量
    bool tf_available_ = false;
    int tf_check_count_ = 0;
    rclcpp::Time last_tf_warn_time_ = this->get_clock()->now();

    // 目标点
    struct Target {
        double x, y, yaw;
    } target1_{0.0, 1.88, 0.0}, target2_{0.0, 0.0, 0.0};

    void step_callback() {
        auto now = this->now();
        // TF可用性判断，流程关键步骤前先判断TF
        if (!tf_available_) {
            if (tf_buffer_.canTransform("odom", "base_link", tf2::TimePointZero)) {
                tf_available_ = true;
                RCLCPP_INFO(this->get_logger(), "TF frames 'odom' 和 'base_link' 现在可用");
            } else {
                tf_check_count_++;
                auto current_time = this->get_clock()->now();
                if (tf_check_count_ <= 5 || (current_time - last_tf_warn_time_).seconds() > 30.0) {
                    RCLCPP_WARN(this->get_logger(), "等待TF frames 'odom' 和 'base_link' 可用... (检查次数: %d)", tf_check_count_);
                    last_tf_warn_time_ = current_time;
                }
                return;
            }
        }
        switch (current_step_) {
            case 0: {
                // 发布car_mode_on
                std_msgs::msg::String cmd;
                cmd.data = "car_mode_on";
                lifecycle_cmd_pub_->publish(cmd);
                RCLCPP_INFO(this->get_logger(), "Step 0: car_mode_on published");
                current_step_ = 1;
                last_time_ = now;
                break;
            }
            case 1: {
                // 等待1s
                if ((now - last_time_).seconds() >= 5.0) {
                    amp_interfaces::msg::TargetPosition target;
                    target.x = target1_.x;
                    target.y = target1_.y;
                    target.yaw = target1_.yaw;
                    target_pub_->publish(target);
                    RCLCPP_INFO(this->get_logger(), "Step 1: target1 published");
                    current_step_ = 2;
                    last_time_ = now;
                }
                break;
            }
            case 2: {
                // 等待进入目标点5cm范围并持续2s
                if (target1_reached_) {
                    if (target_reached_time_ >= 2.0) {
                        std_msgs::msg::String cmd;
                        cmd.data = "fly_mode_on";
                        lifecycle_cmd_pub_->publish(cmd);
                        RCLCPP_INFO(this->get_logger(), "Step 2: fly_mode_on published");
                        current_step_ = 3;
                        last_time_ = now;
                    }
                }
                break;
            }
            case 3: {
                // 等待收到fly_off消息
                if (fly_off_received_) {
                    std_msgs::msg::String cmd;
                    cmd.data = "car_mode_on";
                    lifecycle_cmd_pub_->publish(cmd);
                    RCLCPP_INFO(this->get_logger(), "Step 3: car_mode_on published again");
                    current_step_ = 4;
                    last_time_ = now;
                }
                break;
            }
            case 4: {
                // 等待1s
                if ((now - last_time_).seconds() >= 1.0) {
                    amp_interfaces::msg::TargetPosition target;
                    target.x = target2_.x;
                    target.y = target2_.y;
                    target.yaw = target2_.yaw;
                    target_pub_->publish(target);
                    RCLCPP_INFO(this->get_logger(), "Step 4: target2 published");
                    current_step_ = 5;
                }
                break;
            }
            case 5: {
                // 结束
                break;
            }
        }
    }

    void tf_callback() {
        // 只在第2步检查目标点
        if (current_step_ != 2) return;
        // TF可用性判断
        if (!tf_available_) {
            if (tf_buffer_.canTransform("odom", "base_link", tf2::TimePointZero)) {
                tf_available_ = true;
                RCLCPP_INFO(this->get_logger(), "TF frames 'odom' 和 'base_link' 现在可用");
            } else {
                tf_check_count_++;
                auto current_time = this->get_clock()->now();
                if (tf_check_count_ <= 5 || (current_time - last_tf_warn_time_).seconds() > 30.0) {
                    RCLCPP_WARN(this->get_logger(), "等待TF frames 'odom' 和 'base_link' 可用... (检查次数: %d)", tf_check_count_);
                    last_tf_warn_time_ = current_time;
                }
                return;
            }
        }
        try {
            auto tf = tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);
            double dx = tf.transform.translation.x - target1_.x;
            double dy = tf.transform.translation.y - target1_.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist <= 0.05) {
                if (!target1_reached_) {
                    target1_reached_ = true;
                    target_reached_time_ = 0.0;
                    RCLCPP_INFO(this->get_logger(), "Robot entered target1 area (%.2f m)", dist);
                } else {
                    target_reached_time_ += 0.1; // 每100ms计时
                }
            } else {
                target1_reached_ = false;
                target_reached_time_ = 0.0;
            }
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        }
    }

    void is_off_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "fly_off" && current_step_ == 3) {
            fly_off_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Received fly_off from com_amp");
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RescueTaskNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
