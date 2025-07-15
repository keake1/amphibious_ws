#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>

class LifecycleNodeControl : public rclcpp::Node
{
public:
    LifecycleNodeControl()
        : Node("lifecycle_node_control")
    {
        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/lifecycle_switch_cmd", 10,
            std::bind(&LifecycleNodeControl::cmd_callback, this, std::placeholders::_1));

        // 两个节点的ChangeState客户端
        client_control_ = this->create_client<lifecycle_msgs::srv::ChangeState>("/control_node_lifecycle/change_state");
        client_camera_ = this->create_client<lifecycle_msgs::srv::ChangeState>("/camera_pub_lifecycle/change_state");
        client_temp_ = this->create_client<lifecycle_msgs::srv::ChangeState>("/temp_cam_driver_lifecycle/change_state");
        // 启动时先将两个节点configure
        this->declare_parameter("startup_delay_ms", 500);
        int delay = this->get_parameter("startup_delay_ms").as_int();
        startup_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(delay),
            [this]() {
                this->startup_timer_->cancel();
                this->change_state(client_control_, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, "control_node_lifecycle");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                this->change_state(client_camera_, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, "camera_pub_lifecycle");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                this->change_state(client_temp_, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, "temp_cam_driver_lifecycle");
            }
        );
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_control_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_camera_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_temp_;

    rclcpp::TimerBase::SharedPtr startup_timer_;

    void change_state(
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client,
        uint8_t transition_id,
        const std::string &node_name)
    {
        if (!client->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(get_logger(), "[%s] Lifecycle service not available", node_name.c_str());
            return;
        }
        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->transition.id = transition_id;
        client->async_send_request(req,
            [this, node_name, transition_id](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture fut) {
                auto result = fut.get();
                if (result->success) {
                    RCLCPP_INFO(this->get_logger(), "Node [%s] successfully triggered transition.", node_name.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to trigger transition for node [%s]", node_name.c_str());
                }
            }
        );
    }

    void cmd_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // 收到任何消息都激活两个节点
        if (msg->data == "car_mode_on") 
        {
            RCLCPP_INFO(this->get_logger(), "Activating control_node_lifecycle nodes...");
            change_state(client_control_, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "control_node_lifecycle");
            change_state(client_camera_, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, "camera_pub_lifecycle");
            change_state(client_temp_, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, "temp_cam_driver_lifecycle");
        } 
        else if (msg->data == "fly_mode_on") 
        {
            RCLCPP_INFO(this->get_logger(), "Deactivating control_node_lifecycle nodes...");
            change_state(client_control_, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, "control_node_lifecycle");
            change_state(client_camera_, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "camera_pub_lifecycle");
            change_state(client_temp_, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "temp_cam_driver_lifecycle");
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleNodeControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}