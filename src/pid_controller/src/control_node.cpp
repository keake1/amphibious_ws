#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "pid_controller.hpp"
#include "amp_interfaces/msg/target_position.hpp"
#include <cmath>

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("control_node"), 
                    tf_buffer_(this->get_clock()),
                    tf_listener_(tf_buffer_),
                    pid_x_(1.0, 0.1, 0.01), 
                    pid_y_(1.0, 0.1, 0.01),
                    pid_yaw_(2.0, 0.1, 0.05),
                    wheel_radius_(0.04),  // 轮子半径 4cm (更合理的值)
                    wheel_base_(0.21),    // 轮距 21cm
                    track_width_(0.20) {  // 轴距 20cm
        
        target_position_.x = 0.0;
        target_position_.y = 0.0;
        target_position_.yaw = 0.0; 
        
        target_sub_ = this->create_subscription<amp_interfaces::msg::TargetPosition>(
            "target_position", 10, std::bind(&ControlNode::targetCallback, this, std::placeholders::_1));
        
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("target_velocity", 10);
        wheel_speeds_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speeds", 10);
        
        // 创建定时器定期获取tf变换
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&ControlNode::timerCallback, this));
    }

private:
    void targetCallback(const amp_interfaces::msg::TargetPosition::SharedPtr msg) {
        target_position_ = *msg;
    }

    void timerCallback() {
        try {
            // 获取odom到base_link的变换
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);
            
            // 提取当前位置
            current_position_.x = transform_stamped.transform.translation.x;
            current_position_.y = transform_stamped.transform.translation.y;
            
            // 提取当前姿态角
            tf2::Quaternion q(
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            current_yaw_ = yaw;
            
            // 计算位置误差和角度误差
            double error_x = pid_x_.compute(target_position_.x, current_position_.x, 0.01);
            double error_y = pid_y_.compute(target_position_.y, current_position_.y, 0.01);
            double error_yaw = normalizeAngle(target_position_.yaw - current_yaw_);
            double angular_velocity = pid_yaw_.compute(0.0, error_yaw, 0.01);

            // 发布目标速度
            auto velocity_msg = geometry_msgs::msg::Twist();
            velocity_msg.linear.x = error_x;
            velocity_msg.linear.y = error_y;
            velocity_msg.angular.z = angular_velocity;
            velocity_pub_->publish(velocity_msg);
            
            // 计算麦克纳姆轮速度
            calculateMecanumWheelSpeeds(error_x, error_y, angular_velocity);
            
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
        }
    }

    // 角度归一化函数
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

         // 计算麦克纳姆轮速度
    void calculateMecanumWheelSpeeds(double vx, double vy, double omega) {
        // 麦克纳姆轮运动学逆解
        // 轮子编号: 0-前左, 1-前右, 2-后左, 3-后右
        // 坐标系：前方为x正方向，左方为y正方向，逆时针旋转为正
        double wheel_speeds[4];
        
        // 计算从机器人中心到轮子的距离
        double lx = wheel_base_ / 2.0;   // 前后轮距离的一半
        double ly = track_width_ / 2.0;  // 左右轮距离的一半
        
        // 麦克纳姆轮运动学公式 (标准右手坐标系)
        // 前左轮（45度滚轮）
        wheel_speeds[0] = (vx + vy + omega * (lx + ly)) / wheel_radius_;  // 前左
        // 后左轮（-45度滚轮）
        wheel_speeds[1] = (vx + vy - omega * (lx + ly)) / wheel_radius_;  // 后左
        // 前右轮（-45度滚轮）  
        wheel_speeds[2] = (vx - vy - omega * (lx + ly)) / wheel_radius_;  // 前右
        // 后右轮（45度滚轮）
        wheel_speeds[3] = (vx - vy + omega * (lx + ly)) / wheel_radius_;  // 后右

        // 发布轮子速度
        auto wheel_msg = std_msgs::msg::Float32MultiArray();
        wheel_msg.data.resize(4);
        for (int i = 0; i < 4; i++) {
            wheel_msg.data[i] = wheel_speeds[i];
        }
        wheel_speeds_pub_->publish(wheel_msg);

        // 打印调试信息
        RCLCPP_INFO(this->get_logger(), 
            "Target: vx=%.3f, vy=%.3f, omega=%.3f", vx, vy, omega);
        RCLCPP_INFO(this->get_logger(), 
            "Wheel speeds - FL: %.3f, FR: %.3f, RL: %.3f, RR: %.3f", 
            wheel_speeds[0], wheel_speeds[1], wheel_speeds[2], wheel_speeds[3]);
    }
    // 成员变量
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // PID控制器
    PIDController pid_x_;
    PIDController pid_y_;
    PIDController pid_yaw_;
    
    // 机器人参数
    double wheel_radius_;
    double wheel_base_;
    double track_width_;
    
    // 其他成员变量
    rclcpp::Subscription<amp_interfaces::msg::TargetPosition>::SharedPtr target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speeds_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    amp_interfaces::msg::TargetPosition target_position_;
    geometry_msgs::msg::Point current_position_;
    double current_yaw_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}