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

        client_locate_ = this->create_client<lifecycle_msgs::srv::ChangeState>("/qrcode_locate_lifecycle_node/change_state");
        client_id_detect_ = this->create_client<lifecycle_msgs::srv::ChangeState>("/qrcode_id_detect_lifecycle_node/change_state");

        state_locate_ = this->create_client<lifecycle_msgs::srv::GetState>("/qrcode_locate_lifecycle_node/get_state");
        state_id_detect_ = this->create_client<lifecycle_msgs::srv::GetState>("/qrcode_id_detect_lifecycle_node/get_state");
    }

private:
    // 声明一个用于管理定时器的成员指针
    rclcpp::TimerBase::SharedPtr deactivate_timer_;

    // 订阅指令
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    // 两个节点的Lifecycle客户端
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_locate_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_id_detect_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr state_locate_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr state_id_detect_;

    // 发起ChangeState请求，不阻塞
    void change_state(
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client,
        uint8_t transition_id)
    {
        if (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(get_logger(), "Lifecycle service not available");
            return;
        }
        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->transition.id = transition_id;
        client->async_send_request(req);
    }

    // 异步获取state，拿到后在回调里处理
    void get_and_handle_state(
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr state_client,
        std::function<void(uint8_t)> handle)
    {
        if (!state_client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(get_logger(), "GetState service not available");
            return;
        }
        auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        state_client->async_send_request(
            req,
            [this, handle](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture fut) {
                if (!fut.valid()) {
                    RCLCPP_WARN(get_logger(), "GetState call failed");
                    return;
                }
                auto st = fut.get()->current_state.id;
                RCLCPP_INFO(get_logger(), "异步获取节点状态: %d", st);
                handle(st); // 在回调中执行具体操作
            });
    }

    // 激活节点：异步获取state后，再异步切换
    void activate_node(
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_client,
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr state_client,
        const std::string &node_name)
    {
        get_and_handle_state(state_client, [this, change_client, node_name](uint8_t st) {
            if (st == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
                change_state(change_client, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
                // 可以加一个定时器再次获取状态，当确认变INACTIVE后再activate
                change_state(change_client, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
                RCLCPP_INFO(get_logger(), "已配置并激活[%s]", node_name.c_str());
            } else if (st == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                change_state(change_client, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
                RCLCPP_INFO(get_logger(), "激活[%s]", node_name.c_str());
            } else if (st == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                RCLCPP_INFO(get_logger(), "[%s] 已是激活状态", node_name.c_str());
            } else {
                RCLCPP_WARN(get_logger(), "[%s] 当前无法激活: %d", node_name.c_str(), st);
            }
        });
    }

    // 异步注销节点：先DEACTIVATE，再CLEANUP
    void deactivate_node(
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_client,
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr state_client,
        const std::string &node_name)
    {
        get_and_handle_state(state_client, [this, change_client, node_name, state_client](uint8_t st) {
            if (st == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                change_state(change_client, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
                this->deactivate_timer_ = this->create_wall_timer(  // 加上this->前缀
                    std::chrono::milliseconds(300),
                    [this, change_client, state_client, node_name]() {
                        // 使用this->访问该成员
                        this->deactivate_timer_->cancel();  // 加上this->前缀
                        this->deactivate_timer_.reset();    // 加上this->前缀

                        get_and_handle_state(
                            state_client,
                            [this, change_client, node_name](uint8_t new_state) {
                                if (new_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                                    change_state(change_client, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
                                    RCLCPP_INFO(get_logger(), "已将%s切换为unconfigured状态", node_name.c_str());
                                } else if (new_state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
                                    RCLCPP_INFO(get_logger(), "%s 已经是unconfigured状态", node_name.c_str());
                                } else {
                                    RCLCPP_WARN(get_logger(), "%s 当前无法cleanup: %d", node_name.c_str(), new_state);
                                }
                            }
                        );
                    }
                );
            }
            else if (st == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                // 直接cleanup
                change_state(change_client, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
                RCLCPP_INFO(get_logger(), "已将%s切换为unconfigured状态", node_name.c_str());
            } else if (st == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
                RCLCPP_INFO(get_logger(), "%s 已经是unconfigured状态", node_name.c_str());
            } else {
                RCLCPP_WARN(get_logger(), "%s 当前状态无法cleanup: %d", node_name.c_str(), st);
            }
        });
    }

    void cmd_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "activate_locate") {
            activate_node(client_locate_, state_locate_, "qrcode_locate_lifecycle_node");
            deactivate_node(client_id_detect_, state_id_detect_, "qrcode_id_detect_lifecycle_node");
        } else if (msg->data == "activate_id_detect") {
            activate_node(client_id_detect_, state_id_detect_, "qrcode_id_detect_lifecycle_node");
            deactivate_node(client_locate_, state_locate_, "qrcode_locate_lifecycle_node");
        } else {
            RCLCPP_WARN(this->get_logger(), "收到未知控制指令: %s", msg->data.c_str());
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