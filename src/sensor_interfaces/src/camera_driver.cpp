#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>

class CameraDriver : public rclcpp::Node
{
public:
    CameraDriver() : Node("camera_driver")
    {
        // 声明参数
        this->declare_parameter("video_device", "/dev/video0");
        this->declare_parameter("frame_rate", 30.0);
        this->declare_parameter("width", 640);
        this->declare_parameter("height", 480);
        this->declare_parameter("camera_info_url", "");
        this->declare_parameter("buffer_queue_size", 30);
        
        video_device_ = this->get_parameter("video_device").as_string();
        frame_rate_ = this->get_parameter("frame_rate").as_double();
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        buffer_queue_size_ = this->get_parameter("buffer_queue_size").as_int();
        camera_info_url_ = this->get_parameter("camera_info_url").as_string();

        // 初始化摄像头 - 尝试多种后端
        RCLCPP_INFO(this->get_logger(), "Attempting to open camera: %s", video_device_.c_str());
        
        // 尝试不同的后端
        cap_.open(video_device_, cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            RCLCPP_WARN(this->get_logger(), "Could not open camera with V4L2 backend, trying default...");
            cap_.open(video_device_);
        }
        
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open camera with device: %s", video_device_.c_str());
            return;
        }

        // 设置摄像头参数
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_.set(cv::CAP_PROP_FPS, frame_rate_);
        cap_.set(cv::CAP_PROP_BUFFERSIZE, buffer_queue_size_);
        
        // 尝试设置FOURCC，但不强制要求
        bool fourcc_set = cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        if (!fourcc_set) {
            RCLCPP_WARN(this->get_logger(), "Could not set FOURCC to MJPG, using default");
        }

        // 加载相机参数
        loadCameraInfo();

        // 创建图像发布者
        image_pub_ = image_transport::create_camera_publisher(this, "camera/image_raw");

        // 创建定时器以固定频率发布图像
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / frame_rate_)),
            std::bind(&CameraDriver::publish_image, this)
        );

        RCLCPP_INFO(this->get_logger(), "C100 USB Camera driver initialized successfully on device: %s", video_device_.c_str());
    }

    ~CameraDriver()
    {
        if (cap_.isOpened()) {
            cap_.release();
        }
    }

private:
    void loadCameraInfo()
    {
        if (!camera_info_url_.empty()) {
            try {
                YAML::Node camera_info = YAML::LoadFile(camera_info_url_);
                
                // 解析相机参数
                width_ = camera_info["image_width"].as<int>();
                height_ = camera_info["image_height"].as<int>();
                
                auto K = camera_info["camera_matrix"]["data"].as<std::vector<double>>();
                auto D = camera_info["distortion_coefficients"]["data"].as<std::vector<double>>();
                auto R = camera_info["rectification_matrix"]["data"].as<std::vector<double>>();
                auto P = camera_info["projection_matrix"]["data"].as<std::vector<double>>();
                
                // 填充相机信息
                camera_info_msg_.header.frame_id = "camera_link";
                camera_info_msg_.width = width_;
                camera_info_msg_.height = height_;
                
                std::copy(K.begin(), K.end(), camera_info_msg_.k.begin());
                std::copy(D.begin(), D.end(), camera_info_msg_.d.begin());
                std::copy(R.begin(), R.end(), camera_info_msg_.r.begin());
                std::copy(P.begin(), P.end(), camera_info_msg_.p.begin());
                
                camera_info_msg_.distortion_model = "plumb_bob";
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Could not load camera info from %s: %s", 
                           camera_info_url_.c_str(), e.what());
                // 使用默认参数
                setDefaultCameraInfo();
            }
        } else {
            // 使用默认参数
            setDefaultCameraInfo();
        }
    }
    
    void setDefaultCameraInfo()
    {
        camera_info_msg_.header.frame_id = "camera_link";
        camera_info_msg_.width = width_;
        camera_info_msg_.height = height_;
        camera_info_msg_.distortion_model = "plumb_bob";
        
        // 默认相机内参矩阵 (假设焦距为400px，中心点为图像中心)
        camera_info_msg_.k = {{
            400.0, 0.0, static_cast<double>(width_)/2.0,
            0.0, 400.0, static_cast<double>(height_)/2.0,
            0.0, 0.0, 1.0
        }};
        
        // 无畸变参数
        camera_info_msg_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
        
        // 默认旋转矩阵
        camera_info_msg_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        
        // 默认投影矩阵
        camera_info_msg_.p = {{
            400.0, 0.0, static_cast<double>(width_)/2.0, 0.0,
            0.0, 400.0, static_cast<double>(height_)/2.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        }};
    }

    void publish_image()
    {
        cv::Mat frame;
        if (cap_.read(frame)) {
            if (!frame.empty()) {
                // 转换OpenCV Mat到ROS Image消息
                auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                
                // 设置时间戳
                img_msg->header.stamp = this->now();
                img_msg->header.frame_id = "camera_link";
                
                // 更新相机信息的时间戳
                camera_info_msg_.header.stamp = img_msg->header.stamp;
                
                // 发布图像和相机信息
                // 使用共享指针包装消息
                image_pub_.publish(img_msg, sensor_msgs::msg::CameraInfo::SharedPtr(new sensor_msgs::msg::CameraInfo(camera_info_msg_)));
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read frame from camera. Camera may be disconnected.");
            // 尝试重新连接
            RCLCPP_INFO(this->get_logger(), "Attempting to reconnect to camera...");
            cap_.release();
            
            // 尝试重新打开摄像头
            cap_.open(video_device_, cv::CAP_V4L2);
            if (!cap_.isOpened()) {
                cap_.open(video_device_);
            }
            
            if (cap_.isOpened()) {
                cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
                cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
                cap_.set(cv::CAP_PROP_FPS, frame_rate_);
                cap_.set(cv::CAP_PROP_BUFFERSIZE, buffer_queue_size_);
                
                RCLCPP_INFO(this->get_logger(), "Successfully reconnected to camera");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to reconnect to camera");
            }
        }
    }

    std::string video_device_;
    cv::VideoCapture cap_;
    double frame_rate_;
    int width_, height_, buffer_queue_size_;
    std::string camera_info_url_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    image_transport::CameraPublisher image_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}