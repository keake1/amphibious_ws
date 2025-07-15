#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class Camera0Publisher : public rclcpp::Node
{
public:
    Camera0Publisher()
        : Node("camera_pub")
    {
        pub_cam0_ = this->create_publisher<sensor_msgs::msg::Image>("camera0/image_raw", 10);

        cap0_.open(0,cv::CAP_V4L2);
        // 新增：设置分辨率、帧率、像素格式
        cap0_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap0_.set(cv::CAP_PROP_FRAME_HEIGHT, 640);
        cap0_.set(cv::CAP_PROP_FPS, 25);
        cap0_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','U','Y','V'));
        if (!cap0_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开摄像头0");
            rclcpp::shutdown();
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(40),
            std::bind(&Camera0Publisher::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        cv::Mat frame0, resized0;
        if (cap0_.read(frame0)) {
            cv::resize(frame0, resized0, cv::Size(640, 480));
            auto msg0 = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resized0).toImageMsg();
            msg0->header.stamp = this->now();
            pub_cam0_->publish(*msg0);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_cam0_;
    cv::VideoCapture cap0_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Camera0Publisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}