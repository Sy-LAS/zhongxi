#ifndef SENSOR_INTERFACES_H
#define SENSOR_INTERFACES_H

// 传感器接口主头文件

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

namespace sensor_interfaces
{
    // 传感器接口的基本定义
    class SensorNode : public rclcpp::Node
    {
    public:
        explicit SensorNode(const std::string& node_name);
        virtual ~SensorNode() = default;

    protected:
        // 通用传感器状态
        bool is_initialized_{false};
        bool is_running_{false};
        
        // 传感器参数
        std::string sensor_frame_id_;
        double sensor_publish_rate_{10.0}; // Hz
        
        // 传感器健康检查
        virtual bool check_sensor_health() = 0;
        virtual bool initialize_sensor() = 0;
    };
}

#endif // SENSOR_INTERFACES_H