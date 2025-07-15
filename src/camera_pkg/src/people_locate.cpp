#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/int32.hpp>  // 添加高度消息头文件
#include <memory>
#include <cmath>
#include <string>
#include <vector>

class ItemLocateNode : public rclcpp::Node
{
public:
    ItemLocateNode() : Node("people_locate")
    {
        // 创建TF监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 参数配置
        this->declare_parameter("camera_fov_degrees", 59.5);   // 摄像头水平视场角（度）
        this->declare_parameter("camera_width", 640);          // 摄像头宽度（像素）
        this->declare_parameter("camera_height", 640);         // 摄像头高度（像素）
        this->declare_parameter("camera_offset_x", 0.0);       // 摄像头相对于base_link的X偏移（前方）
        this->declare_parameter("camera_offset_y", 0.0);       // 摄像头相对于base_link的Y偏移（右侧）
        this->declare_parameter("default_height", 160.0);      // 默认高度（厘米）
        this->declare_parameter("min_valid_height", 10.0);     // 最小有效高度（厘米）
        
        camera_fov_degrees_ = this->get_parameter("camera_fov_degrees").as_double();
        camera_width_ = this->get_parameter("camera_width").as_int();
        camera_height_ = this->get_parameter("camera_height").as_int();
        camera_offset_x_ = this->get_parameter("camera_offset_x").as_double();
        camera_offset_y_ = this->get_parameter("camera_offset_y").as_double();
        default_height_ = this->get_parameter("default_height").as_double();
        min_valid_height_ = this->get_parameter("min_valid_height").as_double();
        
        // 初始化高度数据
        current_height_ = default_height_;
        height_valid_ = false;
        
        // 计算像素到世界单位的比例（近似）
        double camera_fov_radians = camera_fov_degrees_ * M_PI / 180.0;
        // 假设相机在1米处的视野宽度
        camera_scale_ = 2.0 * std::tan(camera_fov_radians / 2.0) / camera_width_;
        
        RCLCPP_INFO(this->get_logger(), "摄像头配置：视场角=%.1f°, 分辨率=%dx%d, 比例因子=%.5f",
                    camera_fov_degrees_, camera_width_, camera_height_, camera_scale_);
        
        // 创建检测结果订阅者（修正为订阅人的中心点坐标）
        person_pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/camera0/person_position", 10,
            std::bind(&ItemLocateNode::person_position_callback, this, std::placeholders::_1));
        
        // 创建高度数据订阅者
        height_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/vehicle_height", 10, 
            std::bind(&ItemLocateNode::height_callback, this, std::placeholders::_1));
        
        // 创建物体位置发布者（改为发布geometry_msgs::msg::Point类型，仅x和y）
        item_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/detected_item", 10);
        
        RCLCPP_INFO(this->get_logger(), "物体定位节点已初始化");
        RCLCPP_INFO(this->get_logger(), "订阅高度话题: /vehicle_height (单位: cm)");
    }

private:
    // 高度数据回调函数
    void height_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int height = msg->data;
        
        if (height >= min_valid_height_) {
            current_height_ = static_cast<double>(height);
            height_valid_ = true;
            RCLCPP_DEBUG(this->get_logger(), "接收到高度数据: %.1f cm", current_height_);
        } else {
            // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
            //     "接收到无效高度数据: %d cm，使用默认高度: %.1f cm", height, default_height_);
        }
    }

    // 人的位置回调函数（直接处理中心点像素坐标）
    void person_position_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        try {
            // 获取机器人当前位置
            geometry_msgs::msg::TransformStamped transform = 
                tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
            double robot_x = transform.transform.translation.x;
            double robot_y = transform.transform.translation.y;

            // 使用当前高度计算距离比例因子
            double height_meters = current_height_ / 100.0; // 厘米转米
            double scale_factor = camera_scale_ * height_meters;

            // 获取人的中心点像素坐标
            double center_x = msg->x;
            double center_y = msg->y;

            // 计算中心点相对于画面中心的偏移（以像素为单位）
            double pixel_dx = center_x - camera_width_ / 2.0;
            double pixel_dy = center_y - camera_height_ / 2.0;

            // 坐标转换
            double world_dx = -pixel_dy * scale_factor;
            double world_dy = -pixel_dx * scale_factor;
            double item_x = robot_x + world_dx + camera_offset_x_;
            double item_y = robot_y + world_dy + camera_offset_y_;

            RCLCPP_INFO(this->get_logger(), "检测到人: 位置=(%.2f, %.2f), 当前高度=%.1f cm", item_x, item_y, current_height_);

            geometry_msgs::msg::Point item_point;
            item_point.x = item_x;
            item_point.y = item_y;
            item_point.z = 0.0;
            item_pub_->publish(item_point);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "无法获取变换: %s", ex.what());
        }
    }

    // TF相关变量
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 订阅和发布
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr person_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr height_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr item_pub_;
    
    // 摄像头参数
    double camera_fov_degrees_;
    int camera_width_;
    int camera_height_;
    double camera_scale_;
    double camera_offset_x_;
    double camera_offset_y_;
    
    // 高度参数
    double current_height_;
    double default_height_;
    double min_valid_height_;
    bool height_valid_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ItemLocateNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}