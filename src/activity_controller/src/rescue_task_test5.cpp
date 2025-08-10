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
    RescueTaskNode() : Node("rescue_task_test5"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        lifecycle_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/lifecycle_switch_cmd", 10);
        target_pub_ = this->create_publisher<amp_interfaces::msg::TargetPosition>("/target_position", 10);
        duoji_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/duoji_cmd", 10);  // 新增舵机命令发布器
        
        is_off_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/is_off", 10, std::bind(&RescueTaskNode::is_off_callback, this, std::placeholders::_1));
        
        current_step_ = 0;
        target_reached_time_ = 0.0;
        last_time_ = this->now();
        current_target_reached_ = false;
        target_published_ = false;  // 新增标志位
        
        // 添加启动延迟，确保所有节点都已准备好
        startup_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),  // 延迟2秒启动
            [this]() {
                startup_timer_->cancel();
                // 启动主要定时器
                tf_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RescueTaskNode::tf_callback, this));
                step_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RescueTaskNode::step_callback, this));
                RCLCPP_INFO(this->get_logger(), "RescueTaskNode 定时器已启动，开始任务执行");
            });
        
        RCLCPP_INFO(this->get_logger(), "RescueTaskNode started, waiting for initialization...");
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lifecycle_cmd_pub_;
    rclcpp::Publisher<amp_interfaces::msg::TargetPosition>::SharedPtr target_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr duoji_cmd_pub_;  // 舵机命令发布器
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr step_timer_;
    rclcpp::TimerBase::SharedPtr startup_timer_;  // 新增启动定时器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr is_off_sub_;
    int current_step_;
    rclcpp::Time last_time_;
    double target_reached_time_;
    bool current_target_reached_ = false;
    bool target_published_ = false;  // 新增：标记当前步骤的目标是否已发布
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
                    target_published_ = false;  // 重置目标发布标志
                }
                break;
            }
            case 2: {
                // 第二步：移动至{0.0, -1.9, 0.0} - 索引0
                if (!target_published_) {
                    current_target_ = targets_[0];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 2: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= 2.0) {
                    current_step_ = 3;  // 跳到第3步（原第5步，删除原3、4步）
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                    last_time_ = now;  // 设置时间基准
                }
                break;
            }
            case 3: {
                // 第三步：移动至{3.5, -2.0, 0.0} - 索引1（原第5步）
                if ((now - last_time_).seconds() >= 1.0) {
                    if (!target_published_) {
                        current_target_ = targets_[1];
                        publish_target(current_target_);
                        target_published_ = true;
                        RCLCPP_INFO(this->get_logger(), "Step 3: Moving to target (%.2f, %.2f, %.2f)", 
                                   current_target_.x, current_target_.y, current_target_.yaw);
                    }
                    
                    if (current_target_reached_ && target_reached_time_ >= 2.0) {
                        current_step_ = 4;
                        current_target_reached_ = false;
                        target_reached_time_ = 0.0;
                        target_published_ = false;
                    }
                }
                break;
            }
            case 4: {
                // 第四步：移动至{3.5, -2.0, 1.57} - 索引2（原第6步）
                if (!target_published_) {
                    current_target_ = targets_[2];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 4: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= 2.0) {
                    current_step_ = 5;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 5: {
                // 第五步：移动至{3.5, -1.26, 1.57} - 索引3（原第7步）
                if (!target_published_) {
                    current_target_ = targets_[3];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 5: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= 2.0) {
                    current_step_ = 6;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 6: {
                // 第六步：移动至{3.5, -1.26, 2.1} - 索引4（原第8步）
                if (!target_published_) {
                    current_target_ = targets_[4];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 6: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= 2.0) {
                    current_step_ = 7;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 7: {
                // 第七步：移动至{3.5, -1.26, 1.57} - 索引5（原第9步）
                if (!target_published_) {
                    current_target_ = targets_[5];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 7: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= 2.0) {
                    current_step_ = 8;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 8: {
                // 第八步：移动至{3.5, 0.0, 1.57} - 索引6（原第10步）
                if (!target_published_) {
                    current_target_ = targets_[6];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 8: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= 2.0) {
                    current_step_ = 9;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 9: {
                // 第九步：移动至{3.5, 0.0, -1.57} - 索引7（原第11步）
                if (!target_published_) {
                    current_target_ = targets_[7];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 9: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= 2.0) {
                    current_step_ = 10;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 10: {
                // 第十步：发布"duoji_on"消息到/duoji_cmd（原第12步）
                std_msgs::msg::String duoji_cmd;
                duoji_cmd.data = "duoji_on";
                duoji_cmd_pub_->publish(duoji_cmd);
                RCLCPP_INFO(this->get_logger(), "Step 10: duoji_on published");
                current_step_ = 11;
                target_published_ = false;  // 重置目标发布标志
                break;
            }
            case 11: {
                // 第十一步：移动至{3.5, -0.43, -1.57} - 索引8（原第13步）
                if (!target_published_) {
                    current_target_ = targets_[8];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 11: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= 2.0) {
                    current_step_ = 12;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 12: {
                // 第十二步：移动至{3.5, -0.43, 0.0} - 索引9（原第14步）
                if (!target_published_) {
                    current_target_ = targets_[9];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 12: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= 2.0) {
                    current_step_ = 13;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 13: {
                // 第十三步：移动至{1.6, -0.43, 0.0} - 索引10（原第15步）
                if (!target_published_) {
                    current_target_ = targets_[10];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 13: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= 2.0) {
                    current_step_ = 14;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 14: {
                // 第十四步：移动至{1.6, 0.0, 0.0} - 索引11（原第16步）
                if (!target_published_) {
                    current_target_ = targets_[11];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 14: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= 2.0) {
                    current_step_ = 15;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 15: {
                // 任务完成
                RCLCPP_INFO(this->get_logger(), "Mission completed!");
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
        if (current_step_ != 2 && current_step_ < 3) return;
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
        
        // 只有在目标已发布的情况下才进行到达检测
        if (!target_published_) {
            return;
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
                    RCLCPP_INFO(this->get_logger(), "Robot entered target area (%.2f m) - Target: (%.2f, %.2f)", 
                               dist, current_target_.x, current_target_.y);
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
        // fly_off 相关逻辑已删除，此回调函数保留以防未来需要
        RCLCPP_DEBUG(this->get_logger(), "Received is_off message: %s", msg->data.c_str());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RescueTaskNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}