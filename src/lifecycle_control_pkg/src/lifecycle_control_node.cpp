#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <functional>
#include <random>
#include <thread>

using namespace std::chrono_literals;

class LifecycleManager : public rclcpp::Node 
{
public:
    LifecycleManager() : Node("lifecycle_manager") 
    {
        // 参数配置
        this->declare_parameter("discover_nodes", true);
        this->declare_parameter("service_timeout_sec", 5);
        this->declare_parameter("max_retries", 3);
        this->declare_parameter("node_search_period_sec", 10);
        this->declare_parameter("car_mode_nodes", std::vector<std::string>{
            "/car_drive_pid_pwm_lifecycle_node",
            "/car_pid_lifecycle_node",
            "/velocity_pid_lifecycle_node",
            "/encoder_velocity_lifecycle_node"
        });
        this->declare_parameter("fly_mode_nodes", std::vector<std::string>{
            "/yolo_detect_lifecycle_node"
        });
        
        // 读取参数
        auto_discover_ = this->get_parameter("discover_nodes").as_bool();
        service_timeout_sec_ = this->get_parameter("service_timeout_sec").as_int();
        max_retries_ = this->get_parameter("max_retries").as_int();
        node_search_period_sec_ = this->get_parameter("node_search_period_sec").as_int();
        car_nodes_ = this->get_parameter("car_mode_nodes").as_string_array();
        fly_nodes_ = this->get_parameter("fly_mode_nodes").as_string_array();
        
        // 模式切换订阅
        mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/mode_switch", 10, 
            std::bind(&LifecycleManager::handle_mode_switch, this, std::placeholders::_1));
        
        // 节点发现定时器
        if (auto_discover_) {
            discovery_timer_ = this->create_wall_timer(
                std::chrono::seconds(node_search_period_sec_),
                std::bind(&LifecycleManager::discover_lifecycle_nodes, this));
        }
        
        // 操作处理定时器（适用于需要定时检查的情况，但现在使用回调驱动）
        operation_timer_ = this->create_wall_timer(
            500ms, std::bind(&LifecycleManager::check_pending_operations, this));
            
        RCLCPP_INFO(this->get_logger(), 
            "生命周期管理器已启动 (自动发现=%s, 超时=%ds, 最大重试=%d)",
            auto_discover_ ? "开启" : "关闭", service_timeout_sec_, max_retries_);
            
        // 初始发现
        if (auto_discover_) {
            discover_lifecycle_nodes();
        }
    }

private:
    // 节点状态跟踪
    struct NodeInfo {
        std::string name;
        int current_state = -1;
        int retry_count = 0;
        std::chrono::steady_clock::time_point last_retry;
        bool in_progress = false;
    };
    
    // 操作类型
    enum class OperationType {
        CONFIGURE,
        ACTIVATE,
        DEACTIVATE,
        CLEANUP
    };
    
    // 待处理的操作
    struct PendingOperation {
        std::string node_name;
        OperationType operation;
        int retry_count = 0;
        std::function<void(bool)> callback = nullptr;
        std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now();
    };
    
    // 模式切换处理
    void handle_mode_switch(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到模式切换命令: %s", msg->data.c_str());
        
        if (msg->data == "car_mode_on") {
            activate_mode(car_nodes_, fly_nodes_);
        } else if (msg->data == "fly_mode_on") {
            activate_mode(fly_nodes_, car_nodes_);
        } else {
            RCLCPP_WARN(this->get_logger(), "未知模式: %s", msg->data.c_str());
        }
    }
    
    // 设置指定模式
    void activate_mode(
        const std::vector<std::string>& nodes_to_activate, 
        const std::vector<std::string>& nodes_to_deactivate) 
    {
        // 清空现有操作队列
        operations_.clear();
        
        // 添加启动前日志
        RCLCPP_INFO(this->get_logger(), "开始处理模式切换，要激活 %zu 个节点，要停用 %zu 个节点",
            nodes_to_activate.size(), nodes_to_deactivate.size());
        
        // 打印已知生命周期节点
        std::stringstream known_ss;
        known_ss << "已发现的生命周期节点: ";
        for (const auto& node : known_lifecycle_nodes_) {
            known_ss << node << ", ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", known_ss.str().c_str());
        
        // 先停用不需要的节点
        for (const auto& node : nodes_to_deactivate) {
            deactivate_node(node);
        }
        
        // 再激活需要的节点
        for (const auto& node : nodes_to_activate) {
            activate_node(node);
        }
    }
    
    // 处理待完成的操作
    void check_pending_operations() {
        // 检查是否有超时的操作
        auto now = std::chrono::steady_clock::now();
        
        for (auto it = operations_.begin(); it != operations_.end();) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - it->timestamp).count();
            if (elapsed > service_timeout_sec_) {
                RCLCPP_WARN(this->get_logger(), "操作超时: %s -> %s", 
                    it->node_name.c_str(), operation_type_to_string(it->operation).c_str());
                
                // 如果可以重试，则重新安排
                if (it->retry_count < max_retries_) {
                    it->retry_count++;
                    it->timestamp = now;
                    
                    // 指数退避重试
                    int delay_ms = std::min(1000 * (1 << it->retry_count), 10000);
                    auto timer = this->create_wall_timer(
                        std::chrono::milliseconds(delay_ms),
                        [this, op = *it]() {
                            process_operation(op);
                            return true;  // 一次性定时器
                        });
                    
                    RCLCPP_INFO(this->get_logger(), "重试操作: %s -> %s (尝试 %d/%d)", 
                        it->node_name.c_str(), operation_type_to_string(it->operation).c_str(),
                        it->retry_count, max_retries_);
                        
                    it = operations_.erase(it);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "操作达到最大重试次数，放弃: %s -> %s", 
                        it->node_name.c_str(), operation_type_to_string(it->operation).c_str());
                    it = operations_.erase(it);
                }
            } else {
                ++it;
            }
        }
    }
    
    // 激活节点 - 新的实现方式
    void activate_node(const std::string& node_name) {
        RCLCPP_INFO(this->get_logger(), "开始激活节点: %s", node_name.c_str());
        
        // 使用 get_and_handle_state 获取节点状态后执行相应操作
        get_and_handle_state(node_name, [this, node_name](int state) {
            // 现在state < 0是有效的比较
            if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
                // 未配置 -> 先配置再激活
                RCLCPP_INFO(this->get_logger(), "节点 %s 处于未配置状态，开始配置", node_name.c_str());
                change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, 
                    [this, node_name](bool success) {
                        if (success) {
                            RCLCPP_INFO(this->get_logger(), "节点 %s 配置成功，开始激活", node_name.c_str());
                            // 配置成功后激活
                            change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
                                [this, node_name](bool activate_success) {
                                    if (activate_success) {
                                        RCLCPP_INFO(this->get_logger(), "节点 %s 激活成功", node_name.c_str());
                                    } else {
                                        RCLCPP_ERROR(this->get_logger(), "节点 %s 激活失败", node_name.c_str());
                                    }
                                });
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "节点 %s 配置失败", node_name.c_str());
                        }
                    });
            }
            else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                // 非活动 -> 直接激活
                RCLCPP_INFO(this->get_logger(), "节点 %s 处于非活动状态，开始激活", node_name.c_str());
                change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
                    [this, node_name](bool success) {
                        if (success) {
                            RCLCPP_INFO(this->get_logger(), "节点 %s 激活成功", node_name.c_str());
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "节点 %s 激活失败", node_name.c_str());
                        }
                    });
            }
            else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                // 已激活 -> 无需操作
                RCLCPP_INFO(this->get_logger(), "节点 %s 已处于激活状态", node_name.c_str());
            }
            else if (state < 0) {
                // 错误状态 -> 节点可能不存在
                RCLCPP_ERROR(this->get_logger(), "节点 %s 不可用或状态未知", node_name.c_str());
            }
            else {
                // 其他状态 -> 清理后重新配置和激活
                RCLCPP_INFO(this->get_logger(), "节点 %s 处于其他状态(%d)，执行清理", node_name.c_str(), state);
                change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
                    [this, node_name](bool success) {
                        if (success) {
                            RCLCPP_INFO(this->get_logger(), "节点 %s 清理成功，准备配置", node_name.c_str());
                            // 清理成功后配置
                            change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
                                [this, node_name](bool configure_success) {
                                    if (configure_success) {
                                        RCLCPP_INFO(this->get_logger(), "节点 %s 配置成功，准备激活", node_name.c_str());
                                        // 配置成功后激活
                                        change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
                                            [this, node_name](bool activate_success) {
                                                if (activate_success) {
                                                    RCLCPP_INFO(this->get_logger(), "节点 %s 激活成功", node_name.c_str());
                                                } else {
                                                    RCLCPP_ERROR(this->get_logger(), "节点 %s 激活失败", node_name.c_str());
                                                }
                                            });
                                    } else {
                                        RCLCPP_ERROR(this->get_logger(), "节点 %s 配置失败", node_name.c_str());
                                    }
                                });
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "节点 %s 清理失败", node_name.c_str());
                        }
                    });
            }
        });
    }
    
    // 停用节点 - 新的实现方式
    void deactivate_node(const std::string& node_name) {
        RCLCPP_INFO(this->get_logger(), "开始停用节点: %s", node_name.c_str());
        
        // 使用 get_and_handle_state 获取节点状态后执行相应操作
        get_and_handle_state(node_name, [this, node_name](int state) {
            if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
                // 活动状态 -> 停用
                RCLCPP_INFO(this->get_logger(), "节点 %s 处于活动状态，开始停用", node_name.c_str());
                change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
                    [this, node_name](bool success) {
                        if (success) {
                            RCLCPP_INFO(this->get_logger(), "节点 %s 停用成功", node_name.c_str());
                            
                            // 停用成功后，可以选择性地清理
                            auto timer = this->create_wall_timer(
                                std::chrono::milliseconds(500),
                                [this, node_name]() {
                                    get_and_handle_state(node_name, [this, node_name](uint8_t new_state) {
                                        if (new_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                                            change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
                                                [this, node_name](bool cleanup_success) {
                                                    if (cleanup_success) {
                                                        RCLCPP_INFO(this->get_logger(), "节点 %s 清理成功", node_name.c_str());
                                                    } else {
                                                        RCLCPP_ERROR(this->get_logger(), "节点 %s 清理失败", node_name.c_str());
                                                    }
                                                });
                                        }
                                    });
                                    return true;  // 一次性定时器
                                });
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "节点 %s 停用失败", node_name.c_str());
                        }
                    });
            }
            else if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                // 非活动状态 -> 清理
                RCLCPP_INFO(this->get_logger(), "节点 %s 处于非活动状态，开始清理", node_name.c_str());
                change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
                    [this, node_name](bool success) {
                        if (success) {
                            RCLCPP_INFO(this->get_logger(), "节点 %s 清理成功", node_name.c_str());
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "节点 %s 清理失败", node_name.c_str());
                        }
                    });
            }
            else if (state < 0) {
                // 错误状态 -> 节点可能不存在
                RCLCPP_ERROR(this->get_logger(), "节点 %s 不可用或状态未知", node_name.c_str());
            }
            // 其他状态无需停用
        });
    }
    
    // 处理单个操作 (用于重试逻辑)
    void process_operation(const PendingOperation& op) {
        switch (op.operation) {
            case OperationType::CONFIGURE:
                change_state(op.node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, 
                    [this, op](bool success){
                        if (success) {
                            RCLCPP_INFO(this->get_logger(), "节点 %s 配置成功", op.node_name.c_str());
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "节点 %s 配置失败", op.node_name.c_str());
                        }
                    });
                break;
                
            case OperationType::ACTIVATE:
                change_state(op.node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
                    [this, op](bool success) {
                        if (success) {
                            RCLCPP_INFO(this->get_logger(), "节点 %s 激活成功", op.node_name.c_str());
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "节点 %s 激活失败", op.node_name.c_str());
                        }
                    });
                break;
                
            case OperationType::DEACTIVATE:
                change_state(op.node_name, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
                    [this, op](bool success) {
                        if (success) {
                            RCLCPP_INFO(this->get_logger(), "节点 %s 停用成功", op.node_name.c_str());
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "节点 %s 停用失败", op.node_name.c_str());
                        }
                    });
                break;
                
            case OperationType::CLEANUP:
                change_state(op.node_name, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
                    [this, op](bool success) {
                        if (success) {
                            RCLCPP_INFO(this->get_logger(), "节点 %s 清理成功", op.node_name.c_str());
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "节点 %s 清理失败", op.node_name.c_str());
                        }
                    });
                break;
        }
    }
    
    // 发现生命周期节点
    void discover_lifecycle_nodes() {
        RCLCPP_INFO(this->get_logger(), "正在扫描可用的生命周期节点...");
        
        // 获取所有节点名称
        std::vector<std::string> node_names = get_node_names();
        
        // 检查每个节点是否提供生命周期服务
        for (const auto& node_name : node_names) {
            // 跳过自己
            if (node_name == this->get_fully_qualified_name()) {
                continue;
            }
            
            // 构建正确的服务名称
            std::string get_state_srv = build_service_name(node_name, "get_state");
            
            // 创建临时客户端检查服务可用性
            auto client = this->create_client<lifecycle_msgs::srv::GetState>(get_state_srv);
            
            if (client->service_is_ready()) {
                if (known_lifecycle_nodes_.find(node_name) == known_lifecycle_nodes_.end()) {
                    RCLCPP_INFO(this->get_logger(), "发现新的生命周期节点: %s", node_name.c_str());
                    known_lifecycle_nodes_.insert(node_name);
                }
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "已发现 %zu 个生命周期节点", known_lifecycle_nodes_.size());
    }
    
    // 异步获取状态并处理
    void get_and_handle_state(const std::string& node_name, std::function<void(int)> handle) {
        std::string get_state_srv = build_service_name(node_name, "get_state");
        
        auto client = this->create_client<lifecycle_msgs::srv::GetState>(get_state_srv);
        
        if (!client->wait_for_service(std::chrono::seconds(service_timeout_sec_))) {
            RCLCPP_WARN(this->get_logger(), "服务不可用: %s", get_state_srv.c_str());
            handle(-1);  // 表示服务不可用
            return;
        }
        
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        
        client->async_send_request(request,
            [this, node_name, handle](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
                try {
                    auto response = future.get();
                    uint8_t state = response->current_state.id;
                    RCLCPP_DEBUG(this->get_logger(), "节点 %s 状态: %d", node_name.c_str(), state);
                    handle(state);  // 在回调中处理状态
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "获取状态异常: %s, %s", node_name.c_str(), e.what());
                    handle(-1);  // 表示获取状态出错
                }
            });
    }
    
    // 改变节点状态 - 参考lifecycle_control_node_2的实现
    void change_state(const std::string& node_name, uint8_t transition, 
                     std::function<void(bool)> callback, int retry_count = 0) 
    {
        std::string change_state_srv = build_service_name(node_name, "change_state");
        auto client = this->create_client<lifecycle_msgs::srv::ChangeState>(change_state_srv);
        
        if (!client->wait_for_service(std::chrono::seconds(service_timeout_sec_))) {
            RCLCPP_ERROR(this->get_logger(), "服务不可用: %s", change_state_srv.c_str());
            
            // 智能重试
            if (retry_count < max_retries_) {
                int delay_ms = std::min(1000 * (1 << retry_count), 10000);
                
                RCLCPP_INFO(this->get_logger(), "将在 %d ms 后重试状态转换 (尝试 %d/%d)", 
                           delay_ms, retry_count + 1, max_retries_);
                           
                auto timer = this->create_wall_timer(
                    std::chrono::milliseconds(delay_ms),
                    [this, node_name, transition, callback, retry_count]() {
                        change_state(node_name, transition, callback, retry_count + 1);
                        return true;  // 一次性定时器
                    });
            } else {
                RCLCPP_ERROR(this->get_logger(), "达到最大重试次数，放弃状态转换");
                callback(false);
            }
            return;
        }
        
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;
        
        RCLCPP_DEBUG(this->get_logger(), "发送状态转换请求: %s -> %s", 
                    node_name.c_str(), transition_id_to_string(transition).c_str());
        
        client->async_send_request(request,
            [this, node_name, transition, callback, retry_count](
                rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future) {
                try {
                    auto response = future.get();
                    
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "节点 %s 状态转换 %s 成功", 
                                   node_name.c_str(), transition_id_to_string(transition).c_str());
                        callback(true);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "节点 %s 状态转换 %s 失败", 
                                    node_name.c_str(), transition_id_to_string(transition).c_str());
                        
                        // 重试失败的状态转换
                        if (retry_count < max_retries_) {
                            int delay_ms = std::min(1000 * (1 << retry_count), 10000);
                            
                            RCLCPP_INFO(this->get_logger(), "将在 %d ms 后重试状态转换 (尝试 %d/%d)", 
                                       delay_ms, retry_count + 1, max_retries_);
                                       
                            auto timer = this->create_wall_timer(
                                std::chrono::milliseconds(delay_ms),
                                [this, node_name, transition, callback, retry_count]() {
                                    change_state(node_name, transition, callback, retry_count + 1);
                                    return true;  // 一次性定时器
                                });
                        } else {
                            callback(false);
                        }
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "状态转换异常: %s, %s", node_name.c_str(), e.what());
                    callback(false);
                }
            });
    }
    
    // 获取所有节点名称
    std::vector<std::string> get_node_names() {
        std::vector<std::string> node_names;
        
        try {
            // 使用底层API直接获取完全限定的节点名称
            auto graph_interface = this->get_node_graph_interface();
            if (graph_interface) {
                // 这个API返回的已经是完全限定名称（包含命名空间）
                node_names = graph_interface->get_node_names();
            } else {
                RCLCPP_WARN(this->get_logger(), "无法获取节点图接口");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "获取节点名称失败: %s", e.what());
        }
        
        return node_names;
    }
    
    // 转换操作类型为字符串
    std::string operation_type_to_string(OperationType type) {
        switch (type) {
            case OperationType::CONFIGURE: return "配置";
            case OperationType::ACTIVATE: return "激活";
            case OperationType::DEACTIVATE: return "停用";
            case OperationType::CLEANUP: return "清理";
            default: return "未知";
        }
    }
    
    // 转换转换ID为字符串
    std::string transition_id_to_string(uint8_t id) {
        switch (id) {
            case lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE: return "配置";
            case lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP: return "清理";
            case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE: return "激活";
            case lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE: return "停用";
            case lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN: return "关闭(未配置)";
            case lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN: return "关闭(非活动)";
            case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN: return "关闭(活动)";
            default: return "未知(" + std::to_string(id) + ")";
        }
    }
    
    // 服务名称构建辅助函数
    std::string build_service_name(const std::string& node_name, const std::string& service_name) {
        // 确保节点名称开头有且仅有一个'/'
        std::string normalized_node_name = node_name;
        if (normalized_node_name.empty()) {
            return "";
        }
        
        // 移除开头的'/'，后面再统一添加
        while (!normalized_node_name.empty() && normalized_node_name[0] == '/') {
            normalized_node_name = normalized_node_name.substr(1);
        }
        
        // 确保服务名称开头没有'/'
        std::string normalized_service_name = service_name;
        while (!normalized_service_name.empty() && normalized_service_name[0] == '/') {
            normalized_service_name = normalized_service_name.substr(1);
        }
        
        return "/" + normalized_node_name + "/" + normalized_service_name;
    }
    
    // 成员变量
    bool auto_discover_;
    int service_timeout_sec_;
    int max_retries_;
    int node_search_period_sec_;
    std::vector<std::string> car_nodes_;
    std::vector<std::string> fly_nodes_;
    std::set<std::string> known_lifecycle_nodes_;
    std::vector<PendingOperation> operations_;
    bool is_operation_in_progress_ = false;
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::TimerBase::SharedPtr discovery_timer_;
    rclcpp::TimerBase::SharedPtr operation_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LifecycleManager>());
    rclcpp::shutdown();
    return 0;
}