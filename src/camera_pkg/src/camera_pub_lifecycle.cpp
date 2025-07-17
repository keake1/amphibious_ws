#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using rclcpp_lifecycle::LifecycleNode;
using rclcpp_lifecycle::LifecyclePublisher;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Camera0Publisher : public LifecycleNode
{
public:
    Camera0Publisher()
        : LifecycleNode("camera_pub_lifecycle")
    {}

protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        pub_cam0_ = this->create_publisher<sensor_msgs::msg::Image>("camera0/image_raw", 10);

        cap0_.open(0, cv::CAP_V4L2);
        cap0_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap0_.set(cv::CAP_PROP_FRAME_HEIGHT, 640);
        cap0_.set(cv::CAP_PROP_FPS, 25);
        cap0_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','U','Y','V'));
        if (!cap0_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开摄像头0");
            return CallbackReturn::FAILURE;
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        pub_cam0_->on_activate();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(40),
            std::bind(&Camera0Publisher::timer_callback, this)
        );
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        timer_.reset();
        pub_cam0_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        timer_.reset();
        if (cap0_.isOpened()) {
            cap0_.release();
        }
        pub_cam0_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        timer_.reset();
        if (cap0_.isOpened()) {
            cap0_.release();
        }
        pub_cam0_.reset();
        return CallbackReturn::SUCCESS;
    }

private:
    void timer_callback()
    {
        cv::Mat frame0;
        if (cap0_.read(frame0)) {
            auto msg0 = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame0).toImageMsg();
            msg0->header.stamp = this->now();
            pub_cam0_->publish(*msg0);
        }
    }

    std::shared_ptr<LifecyclePublisher<sensor_msgs::msg::Image>> pub_cam0_;
    cv::VideoCapture cap0_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Camera0Publisher>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}