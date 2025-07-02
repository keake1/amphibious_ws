#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>  // 使用ZBar库代替ArUco
#include <cmath>
#include <vector>
#include <memory>
#include <sstream>

class FOVMeasureNode : public rclcpp::Node
{
public:
    FOVMeasureNode() : Node("fov_measure_node")
    {
        // 声明参数
        this->declare_parameter("qrcode_size", 0.042);    // 二维码实际大小(米)
        this->declare_parameter("camera_topic", "/camera/image_raw");
        this->declare_parameter("min_measurements", 10);  // 最小测量次数
        this->declare_parameter("debug_view", true);      // 是否显示调试视图
        
        // 获取参数
        qrcode_size_ = this->get_parameter("qrcode_size").as_double();
        min_measurements_ = this->get_parameter("min_measurements").as_int();
        debug_view_ = this->get_parameter("debug_view").as_bool();
        
        std::string camera_topic = this->get_parameter("camera_topic").as_string();
        
        // 初始化ZBar扫描器
        scanner_ = new zbar::ImageScanner();
        scanner_->set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
        
        // 创建订阅者
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 10, 
            std::bind(&FOVMeasureNode::imageCallback, this, std::placeholders::_1));
            
        height_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/vehicle_height", 10, 
            std::bind(&FOVMeasureNode::heightCallback, this, std::placeholders::_1));
            
        // 初始化测量数据
        measurements_.clear();
        current_height_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "FOV测量节点已初始化，等待图像和高度数据...");
    }
    
    ~FOVMeasureNode() {
        if (scanner_) {
            delete scanner_;
        }
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // 转换ROS图像消息到OpenCV格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat color_image = cv_ptr->image;
            int image_width = color_image.cols;
            
            // 转换为灰度图像 (ZBar需要)
            cv::Mat gray_image;
            cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);
            
            // 创建ZBar图像
            zbar::Image zbar_image(gray_image.cols, gray_image.rows, "Y800", gray_image.data, gray_image.cols * gray_image.rows);
            
            // 扫描图像中的二维码
            scanner_->scan(zbar_image);
            
            // 检查是否找到二维码且有高度数据
            if (zbar_image.symbol_begin() != zbar_image.symbol_end() && current_height_ > 0) {
                // 获取第一个找到的二维码
                auto symbol = zbar_image.symbol_begin();
                
                // 获取二维码的四个角点
                std::vector<cv::Point2f> qr_corners;
                for (int i = 0; i < symbol->get_location_size(); i++) {
                    qr_corners.push_back(cv::Point2f(symbol->get_location_x(i), symbol->get_location_y(i)));
                }
                
                // 确保我们有四个角点
                if (qr_corners.size() == 4) {
                    // 计算二维码对角线的平均长度（像素）
                    double d1 = cv::norm(qr_corners[0] - qr_corners[2]);
                    double d2 = cv::norm(qr_corners[1] - qr_corners[3]);
                    double qr_pixel_size = (d1 + d2) / 2.0;
                    
                    // 计算实际大小与像素的比例
                    double meters_per_pixel = qrcode_size_ / qr_pixel_size;
                    
                    // 计算整个视场的实际宽度（米）
                    double scene_width_meters = meters_per_pixel * image_width;
                    
                    // 计算视场角（弧度）
                    double height_meters = current_height_ / 100.0;  // 厘米转米
                    double fov_rad = 2.0 * std::atan2(scene_width_meters / 2.0, height_meters);
                    
                    // 转换为角度
                    double fov_deg = fov_rad * 180.0 / M_PI;
                    
                    // 记录测量结果
                    Measurement m;
                    m.height = current_height_;
                    m.fov = fov_deg;
                    measurements_.push_back(m);
                    
                    // 计算平均视场角
                    double avg_fov = 0.0;
                    for (const auto& meas : measurements_) {
                        avg_fov += meas.fov;
                    }
                    avg_fov /= measurements_.size();
                    
                    RCLCPP_INFO(this->get_logger(), 
                        "检测到二维码: %s, 高度: %d cm, 计算FOV: %.2f°, 平均FOV: %.2f° (共%zu个样本)",
                        symbol->get_data().c_str(), current_height_, fov_deg, avg_fov, measurements_.size());
                    
                    // 在图像上绘制结果
                    if (debug_view_) {
                        // 绘制二维码的轮廓
                        std::vector<cv::Point> contour;
                        for (size_t i = 0; i < qr_corners.size(); i++) {
                            contour.push_back(cv::Point(qr_corners[i].x, qr_corners[i].y));
                        }
                        std::vector<std::vector<cv::Point>> contours = {contour};
                        cv::drawContours(color_image, contours, 0, cv::Scalar(0, 255, 0), 2);
                        
                        // 显示二维码的数据
                        cv::putText(color_image, symbol->get_data(), cv::Point(qr_corners[0].x, qr_corners[0].y - 10),
                                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                        
                        // 显示测量结果
                        std::string text = "Height: " + std::to_string(current_height_) + 
                                          " cm, FOV: " + std::to_string(fov_deg).substr(0, 5) + 
                                          "°, Avg: " + std::to_string(avg_fov).substr(0, 5) + "°";
                        cv::putText(color_image, text, cv::Point(10, 30), 
                                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                        
                        cv::imshow("FOV Measurement", color_image);
                        cv::waitKey(1);
                    }
                    
                    // 如果收集了足够的样本，计算最终结果
                    if (measurements_.size() >= static_cast<size_t>(min_measurements_)) {
                        calculateFinalFOV();
                    }
                }
                else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                        "检测到的二维码角点数量不足");
                }
            }
            else {
                if (zbar_image.symbol_begin() == zbar_image.symbol_end()) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                        "未检测到二维码，请确保二维码在视野内");
                }
                if (current_height_ <= 0) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                        "未收到有效高度数据");
                }
                
                if (debug_view_) {
                    cv::imshow("FOV Measurement", color_image);
                    cv::waitKey(1);
                }
            }
            
            // 释放ZBar图像资源
            zbar_image.set_data(NULL, 0);
        }
        catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "处理图像时出错: %s", e.what());
        }
    }
    
    void heightCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        current_height_ = msg->data;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
            "当前高度: %d cm", current_height_);
    }
    
    void calculateFinalFOV()
    {
        // 按高度分组并计算每个高度的平均视场角
        std::map<int, std::vector<double>> height_fov_map;
        for (const auto& m : measurements_) {
            height_fov_map[m.height].push_back(m.fov);
        }
        
        // 计算最终结果
        std::stringstream ss;
        ss << "最终视场角测量结果:\n";
        
        double total_fov = 0.0;
        int sample_count = 0;
        
        for (const auto& [height, fovs] : height_fov_map) {
            double avg_fov = 0.0;
            for (double fov : fovs) {
                avg_fov += fov;
            }
            avg_fov /= fovs.size();
            
            ss << "  高度 " << height << " cm: 平均FOV = " << avg_fov << "° (样本数: " << fovs.size() << ")\n";
            
            total_fov += avg_fov;
            sample_count++;
        }
        
        // 计算所有高度的平均视场角
        double final_fov = total_fov / sample_count;
        ss << "综合多个高度的平均水平视场角: " << final_fov << "°";
        
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }
    
    // 测量数据结构
    struct Measurement {
        int height;    // 高度（cm）
        double fov;    // 视场角（度）
    };
    
    // 成员变量
    zbar::ImageScanner* scanner_;
    double qrcode_size_;  // 米
    int min_measurements_;
    bool debug_view_;
    
    int current_height_;  // 厘米
    std::vector<Measurement> measurements_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr height_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FOVMeasureNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}