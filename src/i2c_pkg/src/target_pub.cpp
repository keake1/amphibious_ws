#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>

using namespace std::chrono_literals;

class TargetPublisherNode : public rclcpp::Node {
public:
    TargetPublisherNode() : Node("target_publisher_node") {
        // 声明参数
        this->declare_parameter("target_x", 0.0);
        this->declare_parameter("target_y", 0.0);
        this->declare_parameter("target_yaw", 0.0);
        this->declare_parameter("publish_rate_ms", 100);
        
        // 读取初始参数值
        target_x_ = this->get_parameter("target_x").as_double();
        target_y_ = this->get_parameter("target_y").as_double();
        target_yaw_ = this->get_parameter("target_yaw").as_double();
        int publish_rate_ms = this->get_parameter("publish_rate_ms").as_int();
        
        // 创建发布者
        target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_position", 10);
        
        // 创建定时器，定期发布目标位置
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_rate_ms),
            std::bind(&TargetPublisherNode::publish_target, this));
        
        // 设置参数回调
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&TargetPublisherNode::parameters_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "目标位置发布节点已初始化，当前目标: x=%.2f, y=%.2f, yaw=%.2f",
                   target_x_, target_y_, target_yaw_);
    }

private:
    // 发布目标位置
    void publish_target() {
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
            }
        }
        
        if (position_changed) {
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
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
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