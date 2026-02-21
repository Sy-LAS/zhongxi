#ifndef LIDAR_INTERFACE_H
#define LIDAR_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_interfaces/sensor_interfaces.h>
#include <memory>
#include <string>
// YDLidar SDK 头文件
#include "sdk/src/ydlidar_driver.h"

namespace sensor_interfaces
{
    class LidarInterface : public SensorNode
    {
    public:
        explicit LidarInterface(const std::string& lidar_name = "lidar_driver");
        ~LidarInterface() override;

        // 启动激光雷达
        bool start_lidar();
        // 停止激光雷达
        bool stop_lidar();

    protected:
        // 传感器接口实现
        bool check_sensor_health() override;
        bool initialize_sensor() override;

    private:
        void publish_scan();
        bool init_lidar_device();

        // 激光雷达参数
        std::string port_;
        int baudrate_{230400};
        std::string frame_id_;
        double range_min_{0.01};
        double range_max_{25.0};
        double angle_min_{-180.0};
        double angle_max_{180.0};
        double frequency_{10.0};
        bool resolution_fixed_{false};
        bool reversion_{true};
        bool auto_reconnect_{true};
        bool intensities_{false};
        bool support_motor_dtr_ctrl_{false};
        int lidar_type_{1};  // TYPE_TRIANGLE
        int device_type_{0}; // YDLIDAR_TYPE_SERIAL
        int sample_rate_{3};

        // YDLidar驱动实例
        ydlidar::YDlidarDriver *lidar_{nullptr};

        // ROS发布者
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
        rclcpp::TimerBase::SharedPtr scan_timer_;
    };
}

#endif // LIDAR_INTERFACE_H