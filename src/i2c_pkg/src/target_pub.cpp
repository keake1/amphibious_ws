#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <vector>
#include <string>

using namespace std::chrono_literals;

// 定义目标点结构
struct Waypoint {
    double x;
    double y;
    double yaw;
};

class TargetPublisherNode : public rclcpp::Node {
public:
    TargetPublisherNode() : Node("target_publisher_node"), current_waypoint_index_(0) {
        // 声明参数
        this->declare_parameter("publish_rate_ms", 100);
        this->declare_parameter("waypoints", std::vector<std::string>({"0.0,0.0,0.0"}));
        this->declare_parameter("loop_waypoints", true);
        this->declare_parameter("auto_start", true);
        
        // 读取参数
        int publish_rate_ms = this->get_parameter("publish_rate_ms").as_int();
        auto waypoint_strings = this->get_parameter("waypoints").as_string_array();
        loop_waypoints_ = this->get_parameter("loop_waypoints").as_bool();
        bool auto_start = this->get_parameter("auto_start").as_bool();
        
        // 解析路径点
        parse_waypoints(waypoint_strings);
        if (waypoints_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "没有有效的路径点，添加默认原点");
            waypoints_.push_back({0.0, 0.0, 0.0});
        }
        
        // 初始化当前目标
        if (auto_start && !waypoints_.empty()) {
            target_x_ = waypoints_[0].x;
            target_y_ = waypoints_[0].y;
            target_yaw_ = waypoints_[0].yaw;
            navigation_active_ = true;
        } else {
            target_x_ = 0.0;
            target_y_ = 0.0;
            target_yaw_ = 0.0;
            navigation_active_ = false;
        }
        
        // 创建发布者
        target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_position", 10);
        
        // 创建目标到达订阅者
        target_reached_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "target_reached", 10, 
            std::bind(&TargetPublisherNode::target_reached_callback, this, std::placeholders::_1));
        
        // 创建定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_rate_ms),
            std::bind(&TargetPublisherNode::publish_target, this));
        
        // 设置参数回调
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&TargetPublisherNode::parameters_callback, this, std::placeholders::_1));
        
        // 打印初始路径点信息
        print_waypoints();
        
        RCLCPP_INFO(this->get_logger(), "目标位置发布节点已初始化，当前目标: x=%.2f, y=%.2f, yaw=%.2f",
                   target_x_, target_y_, target_yaw_);
        if (navigation_active_) {
            RCLCPP_INFO(this->get_logger(), "导航已自动启动，共有%zu个路径点", waypoints_.size());
        } else {
            RCLCPP_INFO(this->get_logger(), "导航未启动，使用ros2 param set设置auto_start参数为true以开始导航");
        }
    }

private:
    // 解析路径点字符串 ("x,y,yaw")
    void parse_waypoints(const std::vector<std::string>& waypoint_strings) {
        waypoints_.clear();
        
        for (const auto& wp_str : waypoint_strings) {
            std::stringstream ss(wp_str);
            std::string token;
            std::vector<double> values;
            
            while (std::getline(ss, token, ',')) {
                try {
                    values.push_back(std::stod(token));
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "解析路径点失败: %s", wp_str.c_str());
                    break;
                }
            }
            
            if (values.size() == 3) {
                waypoints_.push_back({values[0], values[1], values[2]});
            } else {
                RCLCPP_WARN(this->get_logger(), "路径点格式错误: %s，需要三个值 (x,y,yaw)", wp_str.c_str());
            }
        }
    }
    
    // 打印所有路径点信息
    void print_waypoints() {
        RCLCPP_INFO(this->get_logger(), "路径点列表 (共%zu个):", waypoints_.size());
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  [%zu] x=%.2f, y=%.2f, yaw=%.2f",
                       i, waypoints_[i].x, waypoints_[i].y, waypoints_[i].yaw);
        }
    }
    
    // 移动到下一个路径点
    void move_to_next_waypoint() {
        if (waypoints_.empty()) {
            RCLCPP_WARN(this->get_logger(), "没有路径点可以导航");
            navigation_active_ = false;
            return;
        }
        
        current_waypoint_index_++;
        
        // 检查是否到达最后一个路径点
        if (current_waypoint_index_ >= waypoints_.size()) {
            if (loop_waypoints_) {
                RCLCPP_INFO(this->get_logger(), "完成一轮路径导航，循环回到第一个路径点");
                current_waypoint_index_ = 0;
                navigation_completed_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(), "完成所有路径点导航，将继续发送最后目标点");
                // 回退到最后一个有效的路径点索引
                current_waypoint_index_ = waypoints_.size() - 1;
                navigation_completed_ = true;
                // 注意：不再设置 navigation_active_ = false
            }
        }
        
        // 更新当前目标
        target_x_ = waypoints_[current_waypoint_index_].x;
        target_y_ = waypoints_[current_waypoint_index_].y;
        target_yaw_ = waypoints_[current_waypoint_index_].yaw;
        
        RCLCPP_INFO(this->get_logger(), "移动到%s路径点 [%zu/%zu]: x=%.2f, y=%.2f, yaw=%.2f",
                   navigation_completed_ ? "最终" : "下一个",
                   current_waypoint_index_ + 1, waypoints_.size(),
                   target_x_, target_y_, target_yaw_);
    }
    
    // 目标到达回调
    void target_reached_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && navigation_active_) {
            if (navigation_completed_) {
                // 已完成导航，但仍保持在最后一个目标点
                RCLCPP_DEBUG(this->get_logger(), "收到目标到达通知，保持在最终目标点");
            } else {
                // 正常导航中，移动到下一个点
                RCLCPP_INFO(this->get_logger(), "收到目标到达通知，当前路径点 [%zu/%zu] 已完成",
                           current_waypoint_index_ + 1, waypoints_.size());
                move_to_next_waypoint();
                // 立即发布新目标
                publish_target();
            }
        }
    }
    
    // 发布目标位置
    void publish_target() {
        if (!navigation_active_) {
            return;  // 如果导航未激活，不发布目标
        }
        
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "odom";
        
        // 设置位置
        msg.pose.position.x = target_x_;
        msg.pose.position.y = target_y_;
        msg.pose.position.z = 0.0;
        
        // 将yaw角转换为四元数
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, target_yaw_);
        
        // 设置方向
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        
        // 发布消息
        target_pub_->publish(msg);
    }
    
    // 参数变化回调
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        bool position_changed = false;
        
        for (const auto& param : parameters) {
            if (param.get_name() == "target_x") {
                target_x_ = param.as_double();
                position_changed = true;
            } else if (param.get_name() == "target_y") {
                target_y_ = param.as_double();
                position_changed = true;
            } else if (param.get_name() == "target_yaw") {
                target_yaw_ = param.as_double();
                position_changed = true;
            } else if (param.get_name() == "waypoints") {
                auto waypoint_strings = param.as_string_array();
                parse_waypoints(waypoint_strings);
                print_waypoints();
                
                // 重置导航索引
                current_waypoint_index_ = 0;
                if (!waypoints_.empty()) {
                    target_x_ = waypoints_[0].x;
                    target_y_ = waypoints_[0].y;
                    target_yaw_ = waypoints_[0].yaw;
                    position_changed = true;
                    RCLCPP_INFO(this->get_logger(), "路径点已更新，重置到第一个路径点");
                }
            } else if (param.get_name() == "loop_waypoints") {
                loop_waypoints_ = param.as_bool();
                RCLCPP_INFO(this->get_logger(), "路径循环设置已更新: %s", 
                           loop_waypoints_ ? "开启" : "关闭");
            } else if (param.get_name() == "auto_start") {
                navigation_active_ = param.as_bool();
                if (navigation_active_) {
                    navigation_completed_ = false;  // 重置完成标志
                    RCLCPP_INFO(this->get_logger(), "导航状态已更新: 激活");
                    if (!waypoints_.empty()) {
                        current_waypoint_index_ = 0;
                        target_x_ = waypoints_[0].x;
                        target_y_ = waypoints_[0].y;
                        target_yaw_ = waypoints_[0].yaw;
                        position_changed = true;
                        RCLCPP_INFO(this->get_logger(), "导航已启动，从第一个路径点开始");
                    }
                } else {
                    RCLCPP_INFO(this->get_logger(), "导航状态已更新: 停止");
                }
            }
        }
        
        if (position_changed && navigation_active_) {
            RCLCPP_INFO(this->get_logger(), "目标位置已更新: x=%.2f, y=%.2f, yaw=%.2f",
                       target_x_, target_y_, target_yaw_);
            // 立即发布新的目标位置
            publish_target();
        }
        
        return result;
    }
    
    // 成员变量
    double target_x_;
    double target_y_;
    double target_yaw_;
    bool navigation_active_;
    bool navigation_completed_;  // 添加此变量
    bool loop_waypoints_;
    size_t current_waypoint_index_;
    std::vector<Waypoint> waypoints_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_reached_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}