#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraPubNode : public rclcpp::Node {
public:
    CameraPubNode() : Node("camera_pub_node") {
        // 普通通信，不使用 zero-copy
        pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "camera/image_raw", 10);

        // 打开摄像头，设置帧率/分辨率
        cap_.open(0,cv::CAP_V4L2);
        cap_.set(cv::CAP_PROP_FPS, 30);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 640);

        // 以 ~30Hz 频率发布图像
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&CameraPubNode::grabAndPublish, this));
    }

private:
    void grabAndPublish() {
        cv::Mat frame;
        cap_ >> frame;
        if(frame.empty()) return;

        // 转成ROS图像消息
        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        msg->header.stamp = this->now();
        pub_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPubNode>());
    rclcpp::shutdown();
    return 0;
}