#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <vector>
#include <string>
#include "amp_interfaces/srv/set_target.hpp"
#include <std_srvs/srv/trigger.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;

// 路径点结构体
struct Waypoint {
    double x;
    double y;
    double yaw;
};

class TargetPublisherNode : public rclcpp::Node {
public:
    TargetPublisherNode() : Node("target_publisher_node"), current_waypoint_index_(0) {
        // 声明参数
        this->declare_parameter("waypoints", std::vector<std::string>({"0.0,0.0,0.0"}));
        this->declare_parameter("loop_waypoints", true);
        this->declare_parameter("auto_start", true);

        // 读取参数
        auto waypoint_strings = this->get_parameter("waypoints").as_string_array();
        loop_waypoints_ = this->get_parameter("loop_waypoints").as_bool();
        bool auto_start = this->get_parameter("auto_start").as_bool();

        // 解析路径点
        parse_waypoints(waypoint_strings);
        if (waypoints_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "没有有效的路径点，添加默认原点");
            waypoints_.push_back({0.0, 0.0, 0.0});
        }

        // 初始化当前目标
        if (auto_start && !waypoints_.empty()) {
            target_x_ = waypoints_[0].x;
            target_y_ = waypoints_[0].y;
            target_yaw_ = waypoints_[0].yaw;
            navigation_active_ = true;
        } else {
            target_x_ = 0.0;
            target_y_ = 0.0;
            target_yaw_ = 0.0;
            navigation_active_ = false;
        }

        // 创建/set_target服务客户端
        set_target_client_ = this->create_client<amp_interfaces::srv::SetTarget>("/set_target");
        // 创建/notify_arrival服务服务端
        notify_arrival_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/notify_arrival", std::bind(&TargetPublisherNode::notify_arrival_callback, this, std::placeholders::_1, std::placeholders::_2));

        // 参数回调
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&TargetPublisherNode::parameters_callback, this, std::placeholders::_1));

        print_waypoints();
        RCLCPP_INFO(this->get_logger(), "目标位置发布节点已初始化，当前目标: x=%.2f, y=%.2f, yaw=%.2f", target_x_, target_y_, target_yaw_);
        if (navigation_active_) {
            RCLCPP_INFO(this->get_logger(), "导航已自动启动，共有%zu个路径点", waypoints_.size());
            send_current_target();
        } else {
            RCLCPP_INFO(this->get_logger(), "导航未启动，使用ros2 param set设置auto_start参数为true以开始导航");
        }
    }

private:
    // 解析路径点字符串 ("x,y,yaw")
    void parse_waypoints(const std::vector<std::string>& waypoint_strings) {
        waypoints_.clear();
        for (const auto& wp_str : waypoint_strings) {
            std::stringstream ss(wp_str);
            std::string token;
            std::vector<double> values;
            while (std::getline(ss, token, ',')) {
                try {
                    values.push_back(std::stod(token));
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "解析路径点失败: %s", wp_str.c_str());
                    break;
                }
            }
            if (values.size() == 3) {
                waypoints_.push_back({values[0], values[1], values[2]});
            } else {
                RCLCPP_WARN(this->get_logger(), "路径点格式错误: %s，需要三个值 (x,y,yaw)", wp_str.c_str());
            }
        }
    }

    // 打印所有路径点信息
    void print_waypoints() {
        RCLCPP_INFO(this->get_logger(), "路径点列表 (共%zu个):", waypoints_.size());
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  [%zu] x=%.2f, y=%.2f, yaw=%.2f", i, waypoints_[i].x, waypoints_[i].y, waypoints_[i].yaw);
        }
    }

    // 切换到下一个路径点
    void move_to_next_waypoint() {
        if (waypoints_.empty()) {
            RCLCPP_WARN(this->get_logger(), "没有路径点可以导航");
            navigation_active_ = false;
            return;
        }
        current_waypoint_index_++;
        if (current_waypoint_index_ >= waypoints_.size()) {
            if (loop_waypoints_) {
                RCLCPP_INFO(this->get_logger(), "完成一轮路径导航，循环回到第一个路径点");
                current_waypoint_index_ = 0;
            } else {
                RCLCPP_INFO(this->get_logger(), "完成所有路径点导航，保持在最后目标点");
                current_waypoint_index_ = waypoints_.size() - 1;
            }
        }
        target_x_ = waypoints_[current_waypoint_index_].x;
        target_y_ = waypoints_[current_waypoint_index_].y;
        target_yaw_ = waypoints_[current_waypoint_index_].yaw;
        RCLCPP_INFO(this->get_logger(), "切换到路径点 [%zu/%zu]: x=%.2f, y=%.2f, yaw=%.2f", current_waypoint_index_ + 1, waypoints_.size(), target_x_, target_y_, target_yaw_);
    }

    // 发送当前目标到pid_lifecycle
    void send_current_target() {
        if (!navigation_active_ || waypoints_.empty()) return;
        auto req = std::make_shared<amp_interfaces::srv::SetTarget::Request>();
        req->x = target_x_;
        req->y = target_y_;
        req->yaw = target_yaw_;
        RCLCPP_INFO(this->get_logger(), "请求发送目标: x=%.2f, y=%.2f, yaw=%.2f", target_x_, target_y_, target_yaw_);
        
         while (!set_target_client_->wait_for_service(1s)) {
            // RCLCPP_WARN(this->get_logger(), "等待/set_target服务可用...");
        }
        
        RCLCPP_INFO(this->get_logger(), "/set_target服务已可用，发送请求");
        set_target_client_->async_send_request(req, std::bind(&TargetPublisherNode::set_target_response, this, std::placeholders::_1));
    }
    void set_target_response(rclcpp::Client<amp_interfaces::srv::SetTarget>::SharedFuture future) {
        auto res = future.get();
        if (res->success) {
            RCLCPP_INFO(this->get_logger(), "目标已被pid_lifecycle节点接收");
        } else {
            RCLCPP_ERROR(this->get_logger(), "目标发送失败: %s", res->message.c_str());
        }
    }

    // /notify_arrival服务回调，收到到达通知后切换下一个目标并发送
    void notify_arrival_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /* request */,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(this->get_logger(), "收到到达通知，切换下一个目标");
        move_to_next_waypoint();
        send_current_target();
        response->success = true;
        response->message = "收到到达通知";
    }

    // 参数变化回调
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        bool position_changed = false;
        for (const auto& param : parameters) {
            if (param.get_name() == "waypoints") {
                auto waypoint_strings = param.as_string_array();
                parse_waypoints(waypoint_strings);
                print_waypoints();
                current_waypoint_index_ = 0;
                if (!waypoints_.empty()) {
                    target_x_ = waypoints_[0].x;
                    target_y_ = waypoints_[0].y;
                    target_yaw_ = waypoints_[0].yaw;
                    position_changed = true;
                    RCLCPP_INFO(this->get_logger(), "路径点已更新，重置到第一个路径点");
                }
            } else if (param.get_name() == "loop_waypoints") {
                loop_waypoints_ = param.as_bool();
                RCLCPP_INFO(this->get_logger(), "路径循环设置已更新: %s", loop_waypoints_ ? "开启" : "关闭");
            } else if (param.get_name() == "auto_start") {
                navigation_active_ = param.as_bool();
                if (navigation_active_) {
                    RCLCPP_INFO(this->get_logger(), "导航状态已更新: 激活");
                    if (!waypoints_.empty()) {
                        current_waypoint_index_ = 0;
                        target_x_ = waypoints_[0].x;
                        target_y_ = waypoints_[0].y;
                        target_yaw_ = waypoints_[0].yaw;
                        position_changed = true;
                        RCLCPP_INFO(this->get_logger(), "导航已启动，从第一个路径点开始");
                    }
                } else {
                    RCLCPP_INFO(this->get_logger(), "导航状态已更新: 停止");
                }
            }
        }
        if (position_changed && navigation_active_) {
            RCLCPP_INFO(this->get_logger(), "目标位置已更新: x=%.2f, y=%.2f, yaw=%.2f", target_x_, target_y_, target_yaw_);
            send_current_target();
        }
        return result;
    }

    // 成员变量
    double target_x_;
    double target_y_;
    double target_yaw_;
    bool navigation_active_;
    bool loop_waypoints_;
    size_t current_waypoint_index_;
    std::vector<Waypoint> waypoints_;
    rclcpp::Client<amp_interfaces::srv::SetTarget>::SharedPtr set_target_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr notify_arrival_srv_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}