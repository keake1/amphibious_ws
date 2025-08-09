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
    RescueTaskNode() : Node("rescue_task_test4"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        lifecycle_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/lifecycle_switch_cmd", 10);
        target_pub_ = this->create_publisher<amp_interfaces::msg::TargetPosition>("/target_position", 10);
        duoji_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/duoji_cmd", 10);  // 新增舵机命令发布器
        tf_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RescueTaskNode::tf_callback, this));
        step_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RescueTaskNode::step_callback, this));
        is_off_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/is_off", 10, std::bind(&RescueTaskNode::is_off_callback, this, std::placeholders::_1));
        current_step_ = 0;
        target_reached_time_ = 0.0;
        last_time_ = this->now();
        fly_off_received_ = false;
        current_target_reached_ = false;
        RCLCPP_INFO(this->get_logger(), "RescueTaskNode started.");
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lifecycle_cmd_pub_;
    rclcpp::Publisher<amp_interfaces::msg::TargetPosition>::SharedPtr target_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr duoji_cmd_pub_;  // 舵机命令发布器
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr step_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr is_off_sub_;
    int current_step_;
    rclcpp::Time last_time_;
    double target_reached_time_;
    bool fly_off_received_ = false;
    bool current_target_reached_ = false;
    geometry_msgs::msg::TransformStamped current_tf_;
    // TF可用性判断相关变量
    bool tf_available_ = false;
    int tf_check_count_ = 0;
    rclcpp::Time last_tf_warn_time_ = this->get_clock()->now();

    // 当前目标点
    struct Target {
        double x, y, yaw;
    } current_target_;

    // 定义所有目标点
    std::vector<Target> targets_ = {
        {0.0, -1.9, 0.0},      // 索引0 - Step 2
        {3.5, -2.0, 0.0},      // 索引1 - Step 5
        {3.5, -2.0, 1.57},     // 索引2 - Step 6
        {3.5, -1.26, 1.57},    // 索引3 - Step 7
        {3.5, -1.26, 2.1},     // 索引4 - Step 8
        {3.5, -1.26, 1.57},    // 索引5 - Step 9
        {3.5, 0.0, 1.57},      // 索引6 - Step 10
        {3.5, 0.0, -1.57},     // 索引7 - Step 11
        {3.5, -0.43, -1.57},   // 索引8 - Step 13
        {3.5, -0.43, 0.0},     // 索引9 - Step 14
        {1.6, -0.43, 0.0},     // 索引10 - Step 15
        {1.6, 0.0, 0.0}        // 索引11 - Step 16
    };

    void step_callback() {
        auto now = this->now();
        // TF可用性判断，流程关键步骤前先判断TF
        if (!tf_available_ && (current_step_ == 2 || current_step_ >= 5)) {
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
                // 第一步：发布car_mode_on
                std_msgs::msg::String cmd;
                cmd.data = "car_mode_on";
                lifecycle_cmd_pub_->publish(cmd);
                RCLCPP_INFO(this->get_logger(), "Step 0: car_mode_on published");
                current_step_ = 1;
                last_time_ = now;
                break;
            }
            case 1: {
                // 等待5s
                if ((now - last_time_).seconds() >= 5.0) {
                    current_step_ = 2;
                }
                break;
            }
            case 2: {
                // 第二步：移动至{0.0, -1.9, 0.0} - 索引0
                if (!current_target_reached_) {
                    current_target_ = targets_[0];
                    publish_target(current_target_);
                    RCLCPP_INFO(this->get_logger(), "Step 2: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                } else if (target_reached_time_ >= 2.0) {
                    current_step_ = 3;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                }
                break;
            }
            case 3: {
                // 第三步：发布fly_mode_on
                std_msgs::msg::String cmd;
                cmd.data = "fly_mode_on";
                lifecycle_cmd_pub_->publish(cmd);
                RCLCPP_INFO(this->get_logger(), "Step 3: fly_mode_on published");
                current_step_ = 4;
                break;
            }
            case 4: {
                // 第四步：等待收到fly_off消息
                if (fly_off_received_) {
                    std_msgs::msg::String cmd;
                    cmd.data = "car_mode_on";
                    lifecycle_cmd_pub_->publish(cmd);
                    RCLCPP_INFO(this->get_logger(), "Step 4: Received fly_off, switching to car_mode_on");
                    current_step_ = 5;
                    last_time_ = now;
                }
                break;
            }
            case 5: {
                // 第五步：移动至{3.5, -2.0, 0.0} - 索引1
                if ((now - last_time_).seconds() >= 1.0) {
                    if (!current_target_reached_) {
                        current_target_ = targets_[1];
                        publish_target(current_target_);
                        RCLCPP_INFO(this->get_logger(), "Step 5: Moving to target (%.2f, %.2f, %.2f)", 
                                   current_target_.x, current_target_.y, current_target_.yaw);
                    } else if (target_reached_time_ >= 2.0) {
                        current_step_ = 6;
                        current_target_reached_ = false;
                        target_reached_time_ = 0.0;
                    }
                }
                break;
            }
            case 6: {
                // 第六步：移动至{3.5, -2.0, 1.57} - 索引2
                if (!current_target_reached_) {
                    current_target_ = targets_[2];
                    publish_target(current_target_);
                    RCLCPP_INFO(this->get_logger(), "Step 6: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                } else if (target_reached_time_ >= 2.0) {
                    current_step_ = 7;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                }
                break;
            }
            case 7: {
                // 第七步：移动至{3.5, -1.26, 1.57} - 索引3
                if (!current_target_reached_) {
                    current_target_ = targets_[3];
                    publish_target(current_target_);
                    RCLCPP_INFO(this->get_logger(), "Step 7: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                } else if (target_reached_time_ >= 2.0) {
                    current_step_ = 8;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                }
                break;
            }
            case 8: {
                // 第八步：移动至{3.5, -1.26, 2.1} - 索引4
                if (!current_target_reached_) {
                    current_target_ = targets_[4];
                    publish_target(current_target_);
                    RCLCPP_INFO(this->get_logger(), "Step 8: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                } else if (target_reached_time_ >= 2.0) {
                    current_step_ = 9;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                }
                break;
            }
            case 9: {
                // 第九步：移动至{3.5, -1.26, 1.57} - 索引5
                if (!current_target_reached_) {
                    current_target_ = targets_[5];
                    publish_target(current_target_);
                    RCLCPP_INFO(this->get_logger(), "Step 9: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                } else if (target_reached_time_ >= 2.0) {
                    current_step_ = 10;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                }
                break;
            }
            case 10: {
                // 第十步：移动至{3.5, 0.0, 1.57} - 索引6
                if (!current_target_reached_) {
                    current_target_ = targets_[6];
                    publish_target(current_target_);
                    RCLCPP_INFO(this->get_logger(), "Step 10: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                } else if (target_reached_time_ >= 2.0) {
                    current_step_ = 11;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                }
                break;
            }
            case 11: {
                // 第十一步：移动至{3.5, 0.0, -1.57} - 索引7
                if (!current_target_reached_) {
                    current_target_ = targets_[7];
                    publish_target(current_target_);
                    RCLCPP_INFO(this->get_logger(), "Step 11: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                } else if (target_reached_time_ >= 2.0) {
                    current_step_ = 12;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                }
                break;
            }
            case 12: {
                // 第十二步：发布"duoji_on"消息到/duoji_cmd
                std_msgs::msg::String duoji_cmd;
                duoji_cmd.data = "duoji_on";
                duoji_cmd_pub_->publish(duoji_cmd);
                RCLCPP_INFO(this->get_logger(), "Step 12: duoji_on published");
                current_step_ = 13;
                break;
            }
            case 13: {
                // 第十三步：移动至{3.5, -0.43, -1.57} - 索引8
                if (!current_target_reached_) {
                    current_target_ = targets_[8];
                    publish_target(current_target_);
                    RCLCPP_INFO(this->get_logger(), "Step 13: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                } else if (target_reached_time_ >= 2.0) {
                    current_step_ = 14;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                }
                break;
            }
            case 14: {
                // 第十四步：移动至{3.5, -0.43, 0.0} - 索引9
                if (!current_target_reached_) {
                    current_target_ = targets_[9];
                    publish_target(current_target_);
                    RCLCPP_INFO(this->get_logger(), "Step 14: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                } else if (target_reached_time_ >= 2.0) {
                    current_step_ = 15;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                }
                break;
            }
            case 15: {
                // 第十五步：移动至{1.6, -0.43, 0.0} - 索引10
                if (!current_target_reached_) {
                    current_target_ = targets_[10];
                    publish_target(current_target_);
                    RCLCPP_INFO(this->get_logger(), "Step 15: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                } else if (target_reached_time_ >= 2.0) {
                    current_step_ = 16;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                }
                break;
            }
            case 16: {
                // 第十六步：移动至{1.6, 0.0, 0.0} - 索引11
                if (!current_target_reached_) {
                    current_target_ = targets_[11];
                    publish_target(current_target_);
                    RCLCPP_INFO(this->get_logger(), "Step 16: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                } else if (target_reached_time_ >= 2.0) {
                    current_step_ = 17;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                }
                break;
            }
            case 17: {
                // 第十七步：发送fly_mode_on至/lifecycle_switch_cmd
                std_msgs::msg::String cmd;
                cmd.data = "fly_mode_on";
                lifecycle_cmd_pub_->publish(cmd);
                RCLCPP_INFO(this->get_logger(), "Step 17: fly_mode_on published, mission completed");
                current_step_ = 18;
                break;
            }
            case 18: {
                // 任务完成
                break;
            }
        }
    }

    void publish_target(const Target& target) {
        amp_interfaces::msg::TargetPosition target_msg;
        target_msg.x = target.x;
        target_msg.y = target.y;
        target_msg.yaw = target.yaw;
        target_pub_->publish(target_msg);
    }

    void tf_callback() {
        // 只在需要移动的步骤检查目标点
        if (current_step_ != 2 && current_step_ < 5) return;
        if (current_step_ > 17) return;
        
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
            double dx = tf.transform.translation.x - current_target_.x;
            double dy = tf.transform.translation.y - current_target_.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            
            if (dist <= 0.05) {
                if (!current_target_reached_) {
                    current_target_reached_ = true;
                    target_reached_time_ = 0.0;
                    RCLCPP_INFO(this->get_logger(), "Robot entered target area (%.2f m)", dist);
                } else {
                    target_reached_time_ += 0.1; // 每100ms计时
                }
            } else {
                current_target_reached_ = false;
                target_reached_time_ = 0.0;
            }
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        }
    }

    void is_off_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "fly_off" && current_step_ == 4) {
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