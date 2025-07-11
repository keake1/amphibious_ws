#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "amp_interfaces/msg/target_position.hpp"

using namespace std::chrono_literals;

class TargetPublisher : public rclcpp::Node {
public:
  TargetPublisher() : Node("target_pub") {
    this->declare_parameter("x", 1.0);
    this->declare_parameter("y", 0.0);
    this->declare_parameter("yaw", 0.0);

    pub_ = this->create_publisher<amp_interfaces::msg::TargetPosition>("/target_position", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&TargetPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto msg = amp_interfaces::msg::TargetPosition();
    msg.x = this->get_parameter("x").as_double();
    msg.y = this->get_parameter("y").as_double();
    msg.yaw = this->get_parameter("yaw").as_double();
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published target_position: x=%.2f, y=%.2f, yaw=%.2f",
                msg.x, msg.y, msg.yaw);
  }

  rclcpp::Publisher<amp_interfaces::msg::TargetPosition>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetPublisher>());
  rclcpp::shutdown();
  return 0;
}