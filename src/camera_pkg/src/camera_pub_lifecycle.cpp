#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using rclcpp_lifecycle::LifecycleNode;
using rclcpp_lifecycle::LifecyclePublisher;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CameraPubLifecycleNode : public LifecycleNode {
public:
    CameraPubLifecycleNode() : LifecycleNode("camera_pub_lifecycle_node") {
        // 不在构造函数中创建发布者和定时器
    }

protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Configuring...");
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        cap_.open(0, cv::CAP_V4L2);
        cap_.set(cv::CAP_PROP_FPS, 30);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 640);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Activating...");
        pub_->on_activate();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&CameraPubLifecycleNode::grabAndPublish, this));
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Deactivating...");
        timer_.reset();
        pub_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Cleaning up...");
        pub_.reset();
        cap_.release();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Shutting down...");
        timer_.reset();
        pub_.reset();
        cap_.release();
        return CallbackReturn::SUCCESS;
    }

private:
    void grabAndPublish() {
        cv::Mat frame;
        cap_ >> frame;
        if(frame.empty()) return;

        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        pub_->publish(*msg);
    }

    std::shared_ptr<LifecyclePublisher<sensor_msgs::msg::Image>> pub_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPubLifecycleNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}