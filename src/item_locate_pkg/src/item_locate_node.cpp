#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/int32.hpp>  // 添加高度消息头文件
#include <memory>
#include <cmath>
#include <string>
#include <vector>

class ItemLocateNode : public rclcpp::Node
{
public:
    ItemLocateNode() : Node("item_locate_node")
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
        this->declare_parameter("default_height", 100.0);      // 默认高度（厘米）
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
        
        // 创建检测结果订阅者
        detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/camera/detections", 10, 
            std::bind(&ItemLocateNode::detection_callback, this, std::placeholders::_1));
        
        // 创建高度数据订阅者
        height_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/vehicle_height", 10, 
            std::bind(&ItemLocateNode::height_callback, this, std::placeholders::_1));
        
        // 创建物体位置发布者
        item_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
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
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "接收到无效高度数据: %d cm，使用默认高度: %.1f cm", height, default_height_);
        }
    }

    void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
    {
        try {
            // 获取机器人当前位置
            geometry_msgs::msg::TransformStamped transform = 
                tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
            
            // 获取机器人位置和朝向
            double robot_x = transform.transform.translation.x;
            double robot_y = transform.transform.translation.y;
            
            // 从四元数中提取偏航角
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            // 使用当前高度计算距离比例因子
            // 比例 = 高度 * tan(视场角/2) / (像素宽度/2)
            double height_meters = current_height_ / 100.0; // 厘米转米
            double scale_factor = camera_scale_ * height_meters;
            
            // 处理每个检测结果
            for (const auto& detection : msg->detections) {
                // 获取检测框中心点
                double center_x = detection.bbox.center.position.x;
                double center_y = detection.bbox.center.position.y;

                // 获取类别ID和置信度
                std::string class_id_str = detection.results[0].hypothesis.class_id;
                int class_id = std::stoi(class_id_str);
                double score = detection.results[0].hypothesis.score;

                // 计算中心点相对于画面中心的偏移（以像素为单位）
                double pixel_dx = center_x - camera_width_ / 2.0;
                double pixel_dy = center_y - camera_height_ / 2.0;

                // 将像素偏移转换为世界坐标系中的偏移
                // 注意坐标系转换：
                // 摄像头x正方向 → 世界坐标系y负方向
                // 摄像头y正方向 → 世界坐标系x负方向
                double world_dx = -pixel_dy * scale_factor;
                double world_dy = -pixel_dx * scale_factor;

                // 考虑机器人的朝向，计算物体在世界坐标系中的位置
                double item_dx_rotated = world_dx * std::cos(yaw) - world_dy * std::sin(yaw);
                double item_dy_rotated = world_dx * std::sin(yaw) + world_dy * std::cos(yaw);

                // 加上摄像头偏移的修正
                double camera_dx_rotated = camera_offset_x_ * std::cos(yaw) - camera_offset_y_ * std::sin(yaw);
                double camera_dy_rotated = camera_offset_x_ * std::sin(yaw) + camera_offset_y_ * std::cos(yaw);

                // 计算物体的世界坐标
                double item_x = robot_x + item_dx_rotated + camera_dx_rotated;
                double item_y = robot_y + item_dy_rotated + camera_dy_rotated;

                // 日志输出
                RCLCPP_INFO(this->get_logger(), 
                            "检测到物体: 类别=%d, 置信度=%.2f, 位置=(%.2f, %.2f), 当前高度=%.1f cm", 
                            class_id, score, item_x, item_y, current_height_);

                // 创建并发布物体位置
                geometry_msgs::msg::PoseStamped item_pose;
                item_pose.header.frame_id = "odom";
                item_pose.header.stamp = this->now();
                item_pose.pose.position.x = item_x;
                item_pose.pose.position.y = item_y;
                item_pose.pose.position.z = 0.0;
                
                // 设置物体朝向（默认与世界坐标系对齐）
                item_pose.pose.orientation.w = 1.0;
                
                item_pub_->publish(item_pose);
            }
            
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "无法获取变换: %s", ex.what());
        }
    }

    // TF相关变量
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 订阅和发布
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr height_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr item_pub_;
    
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