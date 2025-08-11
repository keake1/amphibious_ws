#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include "amp_interfaces/msg/target_position.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <chrono>
#include <cmath>

class RescueTaskNode : public rclcpp::Node {
public:
    RescueTaskNode() : Node("rescue_task_test4"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // 声明参数
        this->declare_parameter("target_reached_threshold", 2.0);
        this->declare_parameter("position_tolerance", 0.05);
        this->declare_parameter("angle_tolerance", 0.05);
        
        lifecycle_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/lifecycle_switch_cmd", 10);
        target_pub_ = this->create_publisher<amp_interfaces::msg::TargetPosition>("/target_position", 10);
        duoji_cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/duoji_cmd", 10);  // 新增舵机命令发布器
        temp_pos_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/temp_person_positions", 10);

        is_off_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/is_off", 10, std::bind(&RescueTaskNode::is_off_callback, this, std::placeholders::_1));
        
        task_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/task_cmd", 10, std::bind(&RescueTaskNode::task_cmd_callback, this, std::placeholders::_1));

        current_step_ = -1;  // 修改为-1，表示等待task_on命令
        target_reached_time_ = 0.0;
        last_time_ = this->now();
        fly_off_received_ = false;
        current_target_reached_ = false;
        target_published_ = false;  // 新增标志位
        
        // 获取参数值
        target_reached_threshold_ = this->get_parameter("target_reached_threshold").as_double();
        position_tolerance_ = this->get_parameter("position_tolerance").as_double();
        angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
        RCLCPP_INFO(this->get_logger(), "Target reached threshold: %.2f s, Position tolerance: %.3f m, Angle tolerance: %.3f rad (%.1f°)", 
                   target_reached_threshold_, position_tolerance_, angle_tolerance_, angle_tolerance_ * 180.0 / M_PI);
        
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
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr temp_pos_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr step_timer_;
    rclcpp::TimerBase::SharedPtr startup_timer_;  // 新增启动定时器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr is_off_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_cmd_sub_;  // 新增任务命令订阅器
    int current_step_;
    rclcpp::Time last_time_;
    double target_reached_time_;
    double target_reached_threshold_;  // 新增参数变量
    double position_tolerance_;  // 位置容差参数
    double angle_tolerance_;     // 角度容差参数
    bool fly_off_received_ = false;
    bool task_received_ = false;  // 新增：标记是否收到task_on命令
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
        {3.4, -2.1, 0.0},      // 索引1 - Step 5
        {3.4, -2.1, 1.57},     // 索引2 - Step 6
        {3.4, -1.36, 1.57},    // 索引3 - Step 7
        {3.4, -1.36, 2.6},     // 索引4 - Step 8
        {3.4, -1.36, 1.57},    // 索引5 - Step 9
        {3.4, 0.2, 1.57},      // 索引6 - Step 10
        {3.4, 0.2, -1.57},     // 索引7 - Step 11
        {3.4, -0.55, -1.57},   // 索引8 - Step 13
        {3.4, -0.55, 0.0},     // 索引9 - Step 14
        {2.0, -0.55, 0.0},     // 索引10 - Step 15
        {0.0, 0.0, 0.0}        // 索引11 - Step 20 (最终目标)
    };

    void step_callback() {
        auto now = this->now();
        
        // 如果还没收到task_on命令，则不执行任务
        if (!task_received_) {
            return;
        }
        
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
            case -1: {
                // 等待任务开始命令
                RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for task_on command...");
                break;
            }
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
                
                if (current_target_reached_ && target_reached_time_ >= target_reached_threshold_) {
                    current_step_ = 3;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
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
                    current_step_ = 30;  // 转到延时步骤
                    last_time_ = now;
                    target_published_ = false;  // 重置目标发布标志
                }
                break;
            }
            case 30: {
                // 延时步骤：第四步后等待1秒
                if ((now - last_time_).seconds() >= 1.0) {
                    current_step_ = 5;
                    RCLCPP_INFO(this->get_logger(), "Step 4.5: 1 second delay completed");
                }
                break;
            }
            case 5: {
                // 第五步：移动至{3.4, -2.1, 0.0} - 索引1
                if (!target_published_) {
                    current_target_ = targets_[1];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 5: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= target_reached_threshold_) {
                    current_step_ = 6;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 6: {
                // 第六步：移动至{3.4, -2.1, 1.57} - 索引2
                if (!target_published_) {
                    current_target_ = targets_[2];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 6: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= target_reached_threshold_) {
                    current_step_ = 7;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 7: {
                // 第七步：移动至{3.4, -1.36, 1.57} - 索引3
                if (!target_published_) {
                    current_target_ = targets_[3];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 7: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= target_reached_threshold_) {
                    current_step_ = 8;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 8: {
                // 第八步：移动至{3.4, -1.36, 2.6} - 索引4
                if (!target_published_) {
                    current_target_ = targets_[4];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 8: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= target_reached_threshold_) {
                    current_step_ = 9;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 9: {
                // 第九步：移动至{3.4, -1.36, 1.57} - 索引5
                if (!target_published_) {
                    current_target_ = targets_[5];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 9: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= target_reached_threshold_) {
                    current_step_ = 10;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 10: {
                // 第十步：移动至{3.4, 0.15, 1.57} - 索引6
                if (!target_published_) {
                    current_target_ = targets_[6];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 10: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= target_reached_threshold_) {
                    current_step_ = 11;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 11: {
                // 第十一步：移动至{3.4, 0.15, -1.57} - 索引7
                if (!target_published_) {
                    current_target_ = targets_[7];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 11: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= target_reached_threshold_) {
                    current_step_ = 12;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 12: {
                // 第十二步：发布人员位置信息和舵机命令
                auto msg = std_msgs::msg::Float32MultiArray();
                msg.data.resize(3);
                msg.data[0] = 3.4;
                msg.data[1] = 0.0;
                msg.data[2] = 1.0;

                temp_pos_pub_->publish(msg);
                // 发布"duoji_on"消息到/duoji_cmd
                std_msgs::msg::String duoji_cmd;
                duoji_cmd.data = "duoji_on";
                duoji_cmd_pub_->publish(duoji_cmd);
                RCLCPP_INFO(this->get_logger(), "Step 12: duoji_on published");
                current_step_ = 31;  // 转到延时步骤
                last_time_ = now;
                target_published_ = false;  // 重置目标发布标志
                break;
            }
            case 31: {
                // 延时步骤：第十二步后等待1秒
                if ((now - last_time_).seconds() >= 1.0) {
                    current_step_ = 13;
                    RCLCPP_INFO(this->get_logger(), "Step 12.5: 1 second delay completed");
                }
                break;
            }
            case 13: {
                // 第十三步：移动至{3.4, -0.6, -1.57} - 索引8
                if (!target_published_) {
                    current_target_ = targets_[8];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 13: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= target_reached_threshold_) {
                    current_step_ = 14;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 14: {
                // 第十四步：移动至{3.4, -0.6, 0.0} - 索引9
                if (!target_published_) {
                    current_target_ = targets_[9];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 14: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= target_reached_threshold_) {
                    current_step_ = 15;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 15: {
                // 第十五步：移动至{2.0, -0.6, 0.0} - 索引10
                if (!target_published_) {
                    current_target_ = targets_[10];
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 15: Moving to target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= target_reached_threshold_) {
                    current_step_ = 17;  // 直接跳转到第17步，删除第16步
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 17: {
                // 第十七步：发送fly_mode_on至/lifecycle_switch_cmd
                std_msgs::msg::String cmd;
                cmd.data = "fly_mode_on";
                lifecycle_cmd_pub_->publish(cmd);
                RCLCPP_INFO(this->get_logger(), "Step 17: fly_mode_on published");
                current_step_ = 18;
                fly_off_received_ = false;  // 重置fly_off标志
                break;
            }
            case 18: {
                // 第十八步：等待收到fly_off消息
                if (fly_off_received_) {
                    std_msgs::msg::String cmd;
                    cmd.data = "car_mode_on";
                    lifecycle_cmd_pub_->publish(cmd);
                    RCLCPP_INFO(this->get_logger(), "Step 18: Received fly_off, switching to car_mode_on");
                    current_step_ = 19;
                    last_time_ = now;
                    target_published_ = false;  // 重置目标发布标志
                }
                break;
            }
            case 19: {
                // 第十九步：延时1秒
                if ((now - last_time_).seconds() >= 1.0) {
                    current_step_ = 20;
                    RCLCPP_INFO(this->get_logger(), "Step 19: 1 second delay completed");
                }
                break;
            }
            case 20: {
                // 第二十步：移动至{0.0, 0.0, 0.0}
                if (!target_published_) {
                    current_target_.x = 0.0;
                    current_target_.y = 0.0;
                    current_target_.yaw = 0.0;
                    publish_target(current_target_);
                    target_published_ = true;
                    RCLCPP_INFO(this->get_logger(), "Step 20: Moving to final target (%.2f, %.2f, %.2f)", 
                               current_target_.x, current_target_.y, current_target_.yaw);
                }
                
                if (current_target_reached_ && target_reached_time_ >= target_reached_threshold_) {
                    current_step_ = 21;
                    current_target_reached_ = false;
                    target_reached_time_ = 0.0;
                    target_published_ = false;
                }
                break;
            }
            case 21: {
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
        if (current_step_ != 2 && current_step_ < 5) return;
        if (current_step_ > 20) return;  // 更新最大步骤数
        
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
            
            // 计算当前机器人的偏航角
            double current_yaw = std::atan2(
                2.0 * (tf.transform.rotation.w * tf.transform.rotation.z + 
                       tf.transform.rotation.x * tf.transform.rotation.y),
                1.0 - 2.0 * (tf.transform.rotation.y * tf.transform.rotation.y + 
                             tf.transform.rotation.z * tf.transform.rotation.z)
            );
            
            // 计算角度差
            double angle_diff = std::abs(current_yaw - current_target_.yaw);
            // 处理角度环绕问题（-π到π）
            if (angle_diff > M_PI) {
                angle_diff = 2.0 * M_PI - angle_diff;
            }
            
            // 位置和角度都满足条件才认为到达目标
            bool position_reached = (dist <= position_tolerance_);
            bool angle_reached = (angle_diff <= angle_tolerance_);
            
            if (position_reached && angle_reached) {
                if (!current_target_reached_) {
                    current_target_reached_ = true;
                    target_reached_time_ = 0.0;
                    RCLCPP_INFO(this->get_logger(), "Robot reached target - Pos: %.3f m, Angle: %.3f rad (%.1f°) - Target: (%.2f, %.2f, %.2f)", 
                               dist, angle_diff, angle_diff * 180.0 / M_PI, current_target_.x, current_target_.y, current_target_.yaw);
                } else {
                    target_reached_time_ += 0.1; // 每100ms计时
                }
            } else {
                current_target_reached_ = false;
                target_reached_time_ = 0.0;
                // 可选：调试信息，显示当前状态
                if (!position_reached || !angle_reached) {
                    static int debug_count = 0;
                    if (debug_count % 50 == 0) {  // 每5秒输出一次调试信息
                        RCLCPP_DEBUG(this->get_logger(), "Moving to target - Pos: %.3f m (need ≤%.3f), Angle: %.3f rad/%.1f° (need ≤%.3f rad/%.1f°)", 
                                   dist, position_tolerance_, angle_diff, angle_diff * 180.0 / M_PI, 
                                   angle_tolerance_, angle_tolerance_ * 180.0 / M_PI);
                    }
                    debug_count++;
                }
            }
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        }
    }

    void is_off_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "fly_off" && (current_step_ == 4 || current_step_ == 18)) {
            fly_off_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Received fly_off from com_amp at step %d", current_step_);
        }
    }

    void task_cmd_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "task_on" && current_step_ == -1) {
            task_received_ = true;
            current_step_ = 0;  // 开始执行任务
            RCLCPP_INFO(this->get_logger(), "Received task_on command, starting mission...");
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