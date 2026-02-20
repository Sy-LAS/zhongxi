#ifndef CONTROL_BOARD_INTERFACE_H
#define CONTROL_BOARD_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_interfaces/sensor_interfaces.h>
#include <memory>
#include <chrono>
#include <iomanip>
#include <sstream>

// 确保在使用serial库之前正确引入
#ifdef USE_SERIAL_LIB
#include <serial/serial.h>
#endif

namespace sensor_interfaces
{
    class ControlBoardInterface : public SensorNode
    {
    public:
        explicit ControlBoardInterface(const std::string& node_name = "control_board_interface");
        ~ControlBoardInterface();

        // 从基类继承的虚函数
        bool check_sensor_health() override;
        bool initialize_sensor() override;

    private:
        // ROS2订阅者和发布者
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

        // TF broadcaster
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // 串口通信参数
        std::string serial_port_;
        int baud_rate_;
        double wheel_separation_;
        double wheel_radius_;
        double encoder_resolution_;
        double publish_rate_;

        // 里程计参数
        double x_, y_, theta_;
        double vx_, vy_, vtheta_;
        rclcpp::Time last_update_time_;

        // 串口对象
#ifdef USE_SERIAL_LIB
        serial::Serial ser_;
#endif

        // 定时器
        rclcpp::TimerBase::SharedPtr timer_;

        // 回调函数
        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void publish_odom();
        void publish_imu();
        void update_odom_from_encoders(double left_ticks, double right_ticks);
        void broadcast_transforms();
        void parse_serial_data();

        // 串口通信相关
        bool connect_board();
        bool disconnect_board();

        // 参数声明和获取
        void declare_parameters();
    };
}

#endif // CONTROL_BOARD_INTERFACE_H