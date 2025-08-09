#include "com_pkg/serial_com_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<com_pkg::SerialComNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
