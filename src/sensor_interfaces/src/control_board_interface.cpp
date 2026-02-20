#include "sensor_interfaces/control_board_interface.h"
#include <iostream>
#include <cmath>
#include <memory>
#include <iomanip>
#include <sstream>

#ifdef USE_SERIAL_LIB
#include <serial/serial.h>
#endif

namespace sensor_interfaces
{
    ControlBoardInterface::ControlBoardInterface(const std::string& node_name)
        : SensorNode(node_name), 
          x_(0.0), y_(0.0), theta_(0.0)
    {
        declare_parameters();

        // 初始化发布者和订阅者
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&ControlBoardInterface::cmd_vel_callback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

        // 初始化TF广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 获取参数
        this->get_parameter_or("serial_port", serial_port_, std::string("/dev/ttyUSB0"));
        this->get_parameter_or("baud_rate", baud_rate_, 115200);
        this->get_parameter_or("wheel_separation", wheel_separation_, 0.3);
        this->get_parameter_or("wheel_radius", wheel_radius_, 0.05);
        this->get_parameter_or("publish_rate", publish_rate_, 50.0);

#ifdef USE_SERIAL_LIB
        // 初始化串口连接
        ser_.setPort(serial_port_);
        ser_.setBaudrate(baud_rate_);
        ser_.setTimeout(serial::Timeout::simpleTimeout(1000));
        
        try {
            ser_.open();
            RCLCPP_INFO(this->get_logger(), "串口连接成功: %s", serial_port_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "无法打开串口: %s", e.what());
        }
#else
        RCLCPP_WARN(this->get_logger(), "串口库未找到，控制板接口将不会工作");
#endif

        is_initialized_ = true;
        is_running_ = true;

        // 创建定时器发布里程计
        auto timer_callback = [this]() -> void {
            this->publish_odom();
        };
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0/publish_rate_)), 
            timer_callback);

        RCLCPP_INFO(this->get_logger(), "控制板接口初始化完成");
    }

    ControlBoardInterface::~ControlBoardInterface()
    {
#ifdef USE_SERIAL_LIB
        if (ser_.isOpen()) {
            ser_.close();
        }
#endif
        RCLCPP_INFO(this->get_logger(), "控制板接口已关闭");
    }


    void ControlBoardInterface::declare_parameters()
    {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<double>("wheel_separation", 0.3);
        this->declare_parameter<double>("wheel_radius", 0.05);
        this->declare_parameter<double>("publish_rate", 50.0);
    }

    bool ControlBoardInterface::check_sensor_health()
    {
#ifdef USE_SERIAL_LIB
        return ser_.isOpen();
#else
        return false;  // 如果没有串口库，返回false
#endif
    }

    bool ControlBoardInterface::initialize_sensor()
    {
#ifdef USE_SERIAL_LIB
        if (!ser_.isOpen()) {
            RCLCPP_ERROR(this->get_logger(), "串口未打开");
            return false;
        }

        ser_.flush();
        RCLCPP_INFO(this->get_logger(), "传感器初始化完成");
        return true;
#else
        return false;  // 如果没有串口库，返回false
#endif
    }

    void ControlBoardInterface::cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 将速度命令发送到控制板
        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;

#ifdef USE_SERIAL_LIB
        // 构建命令字符串
        std::ostringstream cmd_stream;
        cmd_stream << "#" << std::fixed << std::setprecision(2) 
                  << linear_x << "," << angular_z << "!\n";
        std::string cmd = cmd_stream.str();

        try {
            ser_.write(cmd);
            RCLCPP_DEBUG(this->get_logger(), "发送速度命令: %s", cmd.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "发送命令失败: %s", e.what());
        }
#endif
    }

    void ControlBoardInterface::publish_odom()
    {
#ifdef USE_SERIAL_LIB
        if (!ser_.isOpen()) {
            return;
        }

        if (ser_.available() > 0) {
            parse_serial_data();
        }
#endif

        // 当前时间
        rclcpp::Time current_time = this->now();

        // 构建里程计消息
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";

        // 位置
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        // 方向（四元数）
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0.0, 0.0, theta_);
        geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_tf);
        odom_msg.pose.pose.orientation = quat_msg;

        // 位置协方差
        odom_msg.pose.covariance[0] = 0.01;   // x
        odom_msg.pose.covariance[7] = 0.01;   // y
        odom_msg.pose.covariance[35] = 0.01;  // theta

        // 速度（占位符值）
        odom_msg.twist.twist.linear.x = 0.0;  // 从串口读取实际速度
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;

        // 发布里程计
        odom_pub_->publish(odom_msg);

        // 发布TF变换
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = current_time;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_footprint";
        transform_stamped.transform.translation.x = x_;
        transform_stamped.transform.translation.y = y_;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation = quat_msg;

        tf_broadcaster_->sendTransform(transform_stamped);
    }

    void ControlBoardInterface::parse_serial_data()
    {
#ifdef USE_SERIAL_LIB
        std::string line = ser_.readline();
        if (!line.empty()) {
            // 解析从控制板返回的数据
            // 假设数据格式为: "#x_pos,y_pos,theta,vx,vy,w!"
            if (line[0] == '#' && line[line.length()-1] == '!') {
                line = line.substr(1, line.length()-2);  // 移除首尾字符
                
                std::stringstream ss(line);
                std::string item;
                
                std::vector<double> values;
                while (std::getline(ss, item, ',')) {
                    try {
                        values.push_back(std::stod(item));
                    } catch (const std::invalid_argument& e) {
                        RCLCPP_WARN(this->get_logger(), "解析串口数据失败: %s", item.c_str());
                        return;
                    }
                }
                
                if (values.size() >= 3) {
                    x_ = values[0];
                    y_ = values[1];
                    theta_ = values[2];
                    
                    RCLCPP_DEBUG(this->get_logger(), "解析到位置数据: x=%.2f, y=%.2f, theta=%.2f", 
                               x_, y_, theta_);
                }
            }
        }
#endif
    }

}

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sensor_interfaces::ControlBoardInterface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}