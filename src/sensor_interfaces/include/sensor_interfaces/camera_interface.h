#ifndef CAMERA_INTERFACE_H
#define CAMERA_INTERFACE_H

#include "sensor_interfaces.h"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

namespace sensor_interfaces
{
    class CameraInterface : public SensorNode
    {
    public:
        explicit CameraInterface(const std::string& camera_name = "camera_driver");
        ~CameraInterface() override;

        // 启动摄像头
        bool start_camera();
        // 停止摄像头
        bool stop_camera();

    protected:
        // 传感器接口实现
        bool check_sensor_health() override;
        bool initialize_sensor() override;

    private:
        void publish_image();
        void load_camera_info(const std::string& calibration_file);

        // 摄像头参数
        std::string device_path_;
        int camera_id_{0};
        double frame_rate_{30.0};
        int width_{640};
        int height_{480};
        int buffer_queue_size_{30};
        std::string camera_info_url_;

        // 摄像头对象
        cv::VideoCapture cap_;
        sensor_msgs::msg::CameraInfo camera_info_msg_;

        // 发布者和定时器
        rclcpp::TimerBase::SharedPtr timer_;
        image_transport::CameraPublisher image_pub_;
    };
}

#endif // CAMERA_INTERFACE_H