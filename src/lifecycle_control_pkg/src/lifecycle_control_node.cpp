// 文件名: lifecycle_manager.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class LifecycleManager : public rclcpp::Node {
public:
    LifecycleManager() : Node("lifecycle_manager") {
        // 初始化节点列表
        car_nodes_ = {
            "car_drive_pid_pwm_lifecycle_node", 
            "pid_lifecycle", 
            "velocity_pid_lifecycle_node", 
            "encoder_velocity_lifecycle_node"
        };
        
        fly_nodes_ = {
            "yolo_detect_lifecycle_node"
        };
        
        // 声明服务超时参数
        this->declare_parameter("service_timeout", 3);
        service_timeout_ = this->get_parameter("service_timeout").as_int();
        
        // 订阅模式切换话题
        mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/mode_switch", 10, 
            std::bind(&LifecycleManager::mode_switch_callback, this, std::placeholders::_1));
        
        // 为每个节点创建服务客户端
        initialize_clients();
        
        RCLCPP_INFO(this->get_logger(), "生命周期管理器已初始化");
    }

private:
    void initialize_clients() {
        // 为每个车辆节点创建客户端
        for (const auto& node_name : car_nodes_) {
            std::string change_state_srv = "/" + node_name + "/change_state";
            std::string get_state_srv = "/" + node_name + "/get_state";
            
            change_state_clients_[node_name] = this->create_client<lifecycle_msgs::srv::ChangeState>(change_state_srv);
            get_state_clients_[node_name] = this->create_client<lifecycle_msgs::srv::GetState>(get_state_srv);
            
            RCLCPP_INFO(this->get_logger(), "已创建客户端: %s", change_state_srv.c_str());
        }
        
        // 为YOLO节点创建客户端
        for (const auto& node_name : fly_nodes_) {
            std::string change_state_srv = "/" + node_name + "/change_state";
            std::string get_state_srv = "/" + node_name + "/get_state";
            
            change_state_clients_[node_name] = this->create_client<lifecycle_msgs::srv::ChangeState>(change_state_srv);
            get_state_clients_[node_name] = this->create_client<lifecycle_msgs::srv::GetState>(get_state_srv);
            
            RCLCPP_INFO(this->get_logger(), "已创建客户端: %s", change_state_srv.c_str());
        }
    }
    
    void mode_switch_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到模式切换命令: %s", msg->data.c_str());
        
        if (msg->data == "car_mode_on") {
            // 激活车辆节点，停用YOLO节点
            for (const auto& node_name : car_nodes_) {
                activate_node(node_name);
            }
            
            for (const auto& node_name : fly_nodes_) {
                deactivate_node(node_name);
            }
        } else if (msg->data == "fly_mode_on") {
            // 停用车辆节点，激活YOLO节点
            for (const auto& node_name : car_nodes_) {
                deactivate_node(node_name);
            }
            
            for (const auto& node_name : fly_nodes_) {
                activate_node(node_name);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "未知的模式切换命令: %s", msg->data.c_str());
        }
    }
    
    void get_and_handle_state(const std::string& node_name, 
                             std::function<void(uint8_t)> handle_func) {
        auto client = get_state_clients_[node_name];
        
        if (!client->wait_for_service(std::chrono::seconds(service_timeout_))) {
            RCLCPP_WARN(this->get_logger(), "获取状态服务不可用: %s", node_name.c_str());
            return;
        }
        
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        
        client->async_send_request(request,
            [this, node_name, handle_func](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
                try {
                    auto result = future.get();
                    uint8_t state = result->current_state.id;
                    RCLCPP_INFO(this->get_logger(), "节点 %s 当前状态: %d", node_name.c_str(), state);
                    handle_func(state);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "获取节点状态出错: %s, %s", node_name.c_str(), e.what());
                }
            });
    }
    
    void change_state(const std::string& node_name, uint8_t transition,
                     std::function<void(bool)> callback) {
        auto client = change_state_clients_[node_name];
        
        if (!client->wait_for_service(std::chrono::seconds(service_timeout_))) {
            RCLCPP_WARN(this->get_logger(), "状态转换服务不可用: %s", node_name.c_str());
            callback(false);
            return;
        }
        
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;
        
        RCLCPP_INFO(this->get_logger(), "正在改变节点 %s 状态, 转换ID: %d", node_name.c_str(), transition);
        
        client->async_send_request(request,
            [this, node_name, transition, callback](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future) {
                try {
                    auto result = future.get();
                    if (result->success) {
                        RCLCPP_INFO(this->get_logger(), "成功改变节点 %s 状态, 转换ID: %d", node_name.c_str(), transition);
                        callback(true);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "改变节点 %s 状态失败, 转换ID: %d", node_name.c_str(), transition);
                        callback(false);
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "改变节点状态出错: %s, %s", node_name.c_str(), e.what());
                    callback(false);
                }
            });
    }
    
    void activate_node(const std::string& node_name) {
        RCLCPP_INFO(this->get_logger(), "准备激活节点: %s", node_name.c_str());
        
        get_and_handle_state(node_name, [this, node_name](uint8_t state) {
            if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
                // 未配置状态：先配置再激活
                RCLCPP_INFO(this->get_logger(), "节点 %s 处于未配置状态，开始配置", node_name.c_str());
                change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
                    [this, node_name](bool success) {
                        if (success) {
                            // 延迟200ms再激活，确保配置完成
                            auto timer = this->create_wall_timer(200ms, 
                                [this, node_name]() {
                                    change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
                                        [this, node_name](bool activate_success) {
                                            if (activate_success) {
                                                RCLCPP_INFO(this->get_logger(), "节点 %s 成功激活", node_name.c_str());
                                            } else {
                                                RCLCPP_ERROR(this->get_logger(), "节点 %s 激活失败", node_name.c_str());
                                            }
                                        });
                                    return true;  // 一次性定时器
                                });
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "节点 %s 配置失败", node_name.c_str());
                        }
                    });
            } else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                // 非活动状态：直接激活
                RCLCPP_INFO(this->get_logger(), "节点 %s 处于非活动状态，开始激活", node_name.c_str());
                change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
                    [this, node_name](bool success) {
                        if (success) {
                            RCLCPP_INFO(this->get_logger(), "节点 %s 成功激活", node_name.c_str());
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "节点 %s 激活失败", node_name.c_str());
                        }
                    });
            } else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                // 已激活状态：无需操作
                RCLCPP_INFO(this->get_logger(), "节点 %s 已处于激活状态", node_name.c_str());
            } else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
                // 已终结状态：无法操作
                RCLCPP_ERROR(this->get_logger(), "节点 %s 已终结，无法激活", node_name.c_str());
            } else {
                // 其他状态：先尝试清理，再配置和激活
                RCLCPP_INFO(this->get_logger(), "节点 %s 处于其他状态(%d)，尝试清理后重新激活", node_name.c_str(), state);
                change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
                    [this, node_name](bool success) {
                        if (success) {
                            auto timer = this->create_wall_timer(200ms, 
                                [this, node_name]() {
                                    change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
                                        [this, node_name](bool configure_success) {
                                            if (configure_success) {
                                                auto timer = this->create_wall_timer(200ms, 
                                                    [this, node_name]() {
                                                        change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
                                                            [this, node_name](bool activate_success) {
                                                                if (activate_success) {
                                                                    RCLCPP_INFO(this->get_logger(), "节点 %s 成功激活", node_name.c_str());
                                                                } else {
                                                                    RCLCPP_ERROR(this->get_logger(), "节点 %s 激活失败", node_name.c_str());
                                                                }
                                                            });
                                                        return true;  // 一次性定时器
                                                    });
                                            } else {
                                                RCLCPP_ERROR(this->get_logger(), "节点 %s 配置失败", node_name.c_str());
                                            }
                                        });
                                    return true;  // 一次性定时器
                                });
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "节点 %s 清理失败", node_name.c_str());
                        }
                    });
            }
        });
    }
    
    void deactivate_node(const std::string& node_name) {
        RCLCPP_INFO(this->get_logger(), "准备停用节点: %s", node_name.c_str());
        
        get_and_handle_state(node_name, [this, node_name](uint8_t state) {
            if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                // 活动状态：停用，但不清理
                RCLCPP_INFO(this->get_logger(), "节点 %s 处于活动状态，开始停用", node_name.c_str());
                change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
                    [this, node_name](bool success) {
                        if (success) {
                            RCLCPP_INFO(this->get_logger(), "节点 %s 成功停用", node_name.c_str());
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "节点 %s 停用失败", node_name.c_str());
                        }
                    });
            } 
            else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                // 非活动状态：已经是停用状态，无需操作
                RCLCPP_INFO(this->get_logger(), "节点 %s 已处于非活动状态", node_name.c_str());
            } 
            else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
                // 未配置状态：需要先配置再停用
                RCLCPP_INFO(this->get_logger(), "节点 %s 处于未配置状态，先配置再停用", node_name.c_str());
                change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
                    [this, node_name](bool success) {
                        if (success) {
                            // 配置成功后停用
                            auto timer = this->create_wall_timer(200ms, 
                                [this, node_name]() {
                                    change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
                                        [this, node_name](bool deactivate_success) {
                                            if (deactivate_success) {
                                                RCLCPP_INFO(this->get_logger(), "节点 %s 成功停用", node_name.c_str());
                                            } else {
                                                RCLCPP_ERROR(this->get_logger(), "节点 %s 停用失败", node_name.c_str());
                                            }
                                        });
                                    return true;  // 一次性定时器
                                });
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "节点 %s 配置失败，无法停用", node_name.c_str());
                        }
                    });
            } 
            else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
                // 已终结状态：无法操作
                RCLCPP_ERROR(this->get_logger(), "节点 %s 已终结，无法操作", node_name.c_str());
            } 
            else {
                // 其他状态：先清理，再配置，然后停用
                RCLCPP_INFO(this->get_logger(), "节点 %s 处于其他状态(%d)，尝试清理后配置并停用", node_name.c_str(), state);
                change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
                    [this, node_name](bool success) {
                        if (success) {
                            auto timer = this->create_wall_timer(200ms, 
                                [this, node_name]() {
                                    change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
                                        [this, node_name](bool configure_success) {
                                            if (configure_success) {
                                                auto timer = this->create_wall_timer(200ms, 
                                                    [this, node_name]() {
                                                        change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
                                                            [this, node_name](bool deactivate_success) {
                                                                if (deactivate_success) {
                                                                    RCLCPP_INFO(this->get_logger(), "节点 %s 成功停用", node_name.c_str());
                                                                } else {
                                                                    RCLCPP_ERROR(this->get_logger(), "节点 %s 停用失败", node_name.c_str());
                                                                }
                                                            });
                                                        return true;  // 一次性定时器
                                                    });
                                            } else {
                                                RCLCPP_ERROR(this->get_logger(), "节点 %s 配置失败", node_name.c_str());
                                            }
                                        });
                                    return true;  // 一次性定时器
                                });
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "节点 %s 清理失败", node_name.c_str());
                        }
                    });
            }
        });
    }

    // 成员变量
    std::vector<std::string> car_nodes_;
    std::vector<std::string> fly_nodes_;
    std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> change_state_clients_;
    std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> get_state_clients_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    int service_timeout_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LifecycleManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}