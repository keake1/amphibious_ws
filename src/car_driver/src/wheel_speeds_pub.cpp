#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class WheelSpeedsPublisher : public rclcpp::Node {
public:
  WheelSpeedsPublisher() : Node("wheel_speeds_pub") {
    this->declare_parameter("m1_speed", 500.0);
    this->declare_parameter("m2_speed", 500.0);
    this->declare_parameter("m3_speed", 500.0);
    this->declare_parameter("m4_speed", 500.0);

    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speeds", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&WheelSpeedsPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
    double m1 = this->get_parameter("m1_speed").as_double();
    double m2 = this->get_parameter("m2_speed").as_double();
    double m3 = this->get_parameter("m3_speed").as_double();
    double m4 = this->get_parameter("m4_speed").as_double();
    msg->data = {static_cast<float>(m1), static_cast<float>(m2), static_cast<float>(m3), static_cast<float>(m4)};
    pub_->publish(std::move(msg));
    RCLCPP_INFO(this->get_logger(), "Published wheel speeds: %.2f, %.2f, %.2f, %.2f", m1, m2, m3, m4);
  }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelSpeedsPublisher>());
  rclcpp::shutdown();
  return 0;
}