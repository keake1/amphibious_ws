#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
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

using rclcpp_lifecycle::LifecycleNode;
using rclcpp_lifecycle::LifecyclePublisher;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ControlNode : public LifecycleNode {
public:
    ControlNode() : LifecycleNode("control_node_lifecycle"),
                    tf_buffer_(this->get_clock()),
                    tf_listener_(tf_buffer_),
                    wheel_radius_(0.04),
                    wheel_base_(0.21),
                    track_width_(0.20) {
        // 声明参数并设置默认值
        this->declare_parameter<double>("pid_x_kp", 0.3);
        this->declare_parameter<double>("pid_x_ki", 0.0);
        this->declare_parameter<double>("pid_x_kd", 0.0);
        this->declare_parameter<double>("pid_x_max_output", 0.4);
        this->declare_parameter<double>("pid_x_dead_zone", 0.0);

        this->declare_parameter<double>("pid_y_kp", 0.25);
        this->declare_parameter<double>("pid_y_ki", 0.0);
        this->declare_parameter<double>("pid_y_kd", 0.0);
        this->declare_parameter<double>("pid_y_max_output", 0.4);
        this->declare_parameter<double>("pid_y_dead_zone", 0.0);

        this->declare_parameter<double>("pid_yaw_kp", 0.5);
        this->declare_parameter<double>("pid_yaw_ki", 0.0);
        this->declare_parameter<double>("pid_yaw_kd", 0.0);
        this->declare_parameter<double>("pid_yaw_max_output", 0.8);
        this->declare_parameter<double>("pid_yaw_dead_zone", 0.0);

        target_position_.x = 0.0;
        target_position_.y = 0.0;
        target_position_.yaw = 0.0; 
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        update_pid_params();
        target_sub_ = this->create_subscription<amp_interfaces::msg::TargetPosition>(
            "target_position", 10, std::bind(&ControlNode::targetCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ControlNode::timerCallback, this));
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("target_velocity", 10);
        wheel_speeds_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speeds", 10);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        velocity_pub_->on_activate();
        wheel_speeds_pub_->on_activate();
        RCLCPP_INFO(this->get_logger(), "ControlNode activated.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        velocity_pub_->on_deactivate();
        wheel_speeds_pub_->on_deactivate();
        RCLCPP_INFO(this->get_logger(), "ControlNode deactivated.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        target_sub_.reset();
        timer_.reset();
        velocity_pub_.reset();
        wheel_speeds_pub_.reset();
        RCLCPP_INFO(this->get_logger(), "ControlNode cleaned up.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        target_sub_.reset();
        timer_.reset();
        velocity_pub_.reset();
        wheel_speeds_pub_.reset();
        RCLCPP_INFO(this->get_logger(), "ControlNode shutdown.");
        return CallbackReturn::SUCCESS;
    }

private:
    void update_pid_params() {
        double x_kp = this->get_parameter("pid_x_kp").as_double();
        double x_ki = this->get_parameter("pid_x_ki").as_double();
        double x_kd = this->get_parameter("pid_x_kd").as_double();
        double x_max_output = this->get_parameter("pid_x_max_output").as_double();
        double x_dead_zone = this->get_parameter("pid_x_dead_zone").as_double();

        double y_kp = this->get_parameter("pid_y_kp").as_double();
        double y_ki = this->get_parameter("pid_y_ki").as_double();
        double y_kd = this->get_parameter("pid_y_kd").as_double();
        double y_max_output = this->get_parameter("pid_y_max_output").as_double();
        double y_dead_zone = this->get_parameter("pid_y_dead_zone").as_double();

        double yaw_kp = this->get_parameter("pid_yaw_kp").as_double();
        double yaw_ki = this->get_parameter("pid_yaw_ki").as_double();
        double yaw_kd = this->get_parameter("pid_yaw_kd").as_double();
        double yaw_max_output = this->get_parameter("pid_yaw_max_output").as_double();
        double yaw_dead_zone = this->get_parameter("pid_yaw_dead_zone").as_double();

        pid_x_.set_params(x_kp, x_ki, x_kd, x_max_output, x_dead_zone);
        pid_y_.set_params(y_kp, y_ki, y_kd, y_max_output, y_dead_zone);
        pid_yaw_.set_params(yaw_kp, yaw_ki, yaw_kd, yaw_max_output, yaw_dead_zone);

        RCLCPP_INFO(this->get_logger(), "PID参数已更新: x[%.3f, %.3f, %.3f, max=%.2f, dead=%.3f] y[%.3f, %.3f, %.3f, max=%.2f, dead=%.3f] yaw[%.3f, %.3f, %.3f, max=%.2f, dead=%.3f]",
            x_kp, x_ki, x_kd, x_max_output, x_dead_zone,
            y_kp, y_ki, y_kd, y_max_output, y_dead_zone,
            yaw_kp, yaw_ki, yaw_kd, yaw_max_output, yaw_dead_zone);
    }

    void targetCallback(const amp_interfaces::msg::TargetPosition::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received target position: x=%.2f, y=%.2f, yaw=%.2f",
                    msg->x, msg->y, msg->yaw);
        target_position_ = *msg;
    }

    void timerCallback() {
        try {
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);
            
            current_position_.x = transform_stamped.transform.translation.x;
            current_position_.y = transform_stamped.transform.translation.y;
            
            tf2::Quaternion q(
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            current_yaw_ = yaw;
            
            double error_x = pid_x_.compute(target_position_.x, current_position_.x, 0.01);
            double error_y = pid_y_.compute(target_position_.y, current_position_.y, 0.01);
            double error_yaw = normalizeAngle(target_position_.yaw - current_yaw_);
            double angular_velocity = pid_yaw_.compute(error_yaw, 0.0, 0.01);

            auto velocity_msg = geometry_msgs::msg::Twist();
            velocity_msg.linear.x = error_x;
            velocity_msg.linear.y = error_y;
            velocity_msg.angular.z = angular_velocity;
            velocity_pub_->publish(velocity_msg);
            
            calculateMecanumWheelSpeeds(error_x, error_y, angular_velocity);
            
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
        }
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void calculateMecanumWheelSpeeds(double vx, double vy, double omega) {
        double wheel_speeds[4];
        double lx = wheel_base_ / 2.0;
        double ly = track_width_ / 2.0;
        wheel_speeds[0] = (vx - vy - omega * (lx + ly));
        wheel_speeds[1] = (vx + vy - omega * (lx + ly));
        wheel_speeds[2] = (vx + vy + omega * (lx + ly));
        wheel_speeds[3] = (vx - vy + omega * (lx + ly));
        auto wheel_msg = std_msgs::msg::Float32MultiArray();
        wheel_msg.data.resize(4);
        for (int i = 0; i < 4; i++) {
            wheel_msg.data[i] = wheel_speeds[i];
        }
        wheel_speeds_pub_->publish(wheel_msg);

        // RCLCPP_INFO(this->get_logger(), 
        //     "Target: vx=%.3f, vy=%.3f, omega=%.3f", vx, vy, omega);
        // RCLCPP_INFO(this->get_logger(), 
        //     "Wheel speeds - FL: %.3f, FR: %.3f, RL: %.3f, RR: %.3f", 
        //     wheel_speeds[0], wheel_speeds[1], wheel_speeds[2], wheel_speeds[3]);
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    PIDController pid_x_{0.3, 0.0, 0.0, 1.0, 0.0};  // 初始化值，后续会通过参数覆盖
    PIDController pid_y_{0.25, 0.0, 0.0, 1.0, 0.0};
    PIDController pid_yaw_{0.5, 0.0, 0.0, 0.8, 0.0};
    double wheel_radius_;
    double wheel_base_;
    double track_width_;
    rclcpp::Subscription<amp_interfaces::msg::TargetPosition>::SharedPtr target_sub_;
    std::shared_ptr<LifecyclePublisher<geometry_msgs::msg::Twist>> velocity_pub_;
    std::shared_ptr<LifecyclePublisher<std_msgs::msg::Float32MultiArray>> wheel_speeds_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    amp_interfaces::msg::TargetPosition target_position_;
    geometry_msgs::msg::Point current_position_;
    double current_yaw_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}