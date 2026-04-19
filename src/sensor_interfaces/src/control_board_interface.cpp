#include "sensor_interfaces/control_board_interface.h"
#include "sensor_interfaces/stm32_protocol.h"
#include "sensor_interfaces/stm32_comms_manager.h"
#include <iostream>
#include <cmath>
#include <memory>
#include <iomanip>
#include <sstream>

namespace sensor_interfaces
{
    ControlBoardInterface::ControlBoardInterface(const std::string& node_name)
        : SensorNode(node_name), 
          x_(0.0), y_(0.0), theta_(0.0), vx_(0.0), vy_(0.0), vtheta_(0.0)
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
        std::string serial_port;
        int baud_rate;
        this->get_parameter_or("serial_port", serial_port, std::string("/dev/ttyUSB0"));
        this->get_parameter_or("baud_rate", baud_rate, 115200);
        this->get_parameter_or("wheel_separation", wheel_separation_, 0.3);
        this->get_parameter_or("wheel_radius", wheel_radius_, 0.05);
        this->get_parameter_or("publish_rate", publish_rate_, 50.0);
        this->get_parameter_or("encoder_resolution", encoder_resolution_, 400.0); // JGA25-370电机编码器分辨率

        // 创建串口通信管理器
        comms_manager_ = std::make_shared<Stm32CommsManager>(serial_port, baud_rate);
        
        // 尝试连接到STM32
        if (comms_manager_->connect()) {
            RCLCPP_INFO(this->get_logger(), "成功连接到STM32控制板");
            
            // 发送重置里程计命令
            std::string reset_cmd = Stm32Protocol::buildResetOdomCommand();
            comms_manager_->sendCommand(reset_cmd);
            RCLCPP_INFO(this->get_logger(), "已发送重置里程计命令");
        } else {
            RCLCPP_ERROR(this->get_logger(), "无法连接到STM32控制板，请检查串口连接和参数设置");
        }

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
        if (comms_manager_) {
            // 发送停止命令
            std::string stop_cmd = Stm32Protocol::buildMoveCommand(0.0, 0.0);
            comms_manager_->sendCommand(stop_cmd);
            comms_manager_->disconnect();
        }
        RCLCPP_INFO(this->get_logger(), "控制板接口已关闭");
    }


    void ControlBoardInterface::declare_parameters()
    {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<double>("wheel_separation", 0.3);
        this->declare_parameter<double>("wheel_radius", 0.05);
        this->declare_parameter<double>("encoder_resolution", 400.0);
        this->declare_parameter<double>("publish_rate", 50.0);
    }

    bool ControlBoardInterface::check_sensor_health()
    {
        if (comms_manager_) {
            return comms_manager_->isConnected();
        }
        return false;
    }

    bool ControlBoardInterface::initialize_sensor()
    {
        if (!comms_manager_) {
            RCLCPP_ERROR(this->get_logger(), "串口通信管理器未初始化");
            return false;
        }

        if (!comms_manager_->isConnected()) {
            RCLCPP_ERROR(this->get_logger(), "串口未连接");
            return false;
        }

        // 发送重置命令
        std::string reset_cmd = Stm32Protocol::buildResetOdomCommand();
        comms_manager_->sendCommand(reset_cmd);
        
        RCLCPP_INFO(this->get_logger(), "传感器初始化完成");
        return true;
    }

    void ControlBoardInterface::cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 将速度命令发送到控制板
        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;

        if (comms_manager_ && comms_manager_->isConnected()) {
            // 使用协议构建命令字符串
            std::string cmd = Stm32Protocol::buildMoveCommand(linear_x, angular_z);

            if (comms_manager_->sendCommand(cmd)) {
                RCLCPP_DEBUG(this->get_logger(), "发送速度命令: %s", cmd.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "发送命令失败");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "STM32未连接，无法发送命令");
        }
    }

    void ControlBoardInterface::publish_odom()
    {
        if (!comms_manager_ || !comms_manager_->isConnected()) {
            return;
        }

        // 尝试读取串口数据
        std::string received_data = comms_manager_->readData(100); // 100ms超时
        if (!received_data.empty()) {
            parse_serial_data(received_data);
        }

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

        // 位置协方差（根据传感器精度调整）
        odom_msg.pose.covariance[0] = 0.01;   // x
        odom_msg.pose.covariance[7] = 0.01;   // y
        odom_msg.pose.covariance[35] = 0.015;  // theta

        // 速度（从串口获取实际速度值，当前使用占位符）
        odom_msg.twist.twist.linear.x = vx_;
        odom_msg.twist.twist.linear.y = vy_;
        odom_msg.twist.twist.angular.z = vtheta_;

        // 速度协方差
        odom_msg.twist.covariance[0] = 0.001;   // vx
        odom_msg.twist.covariance[7] = 0.001;   // vy
        odom_msg.twist.covariance[35] = 0.001;  // vtheta

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

    void ControlBoardInterface::parse_serial_data(const std::string& data)
    {
        // 解析从控制板返回的数据
        std::vector<double> values = Stm32Protocol::parseResponse(data);
        
        if (values.size() >= 3) {
            // 更新位置信息 (x, y, theta)
            x_ = values[0];
            y_ = values[1];
            theta_ = values[2];
            
            RCLCPP_DEBUG(this->get_logger(), "解析到位置数据: x=%.3f, y=%.3f, theta=%.3f", 
                       x_, y_, theta_);
        }
        
        if (values.size() >= 6) {
            // 更新速度信息 (vx, vy, vtheta)
            vx_ = values[3];
            vy_ = values[4];
            vtheta_ = values[5];
            
            RCLCPP_DEBUG(this->get_logger(), "解析到速度数据: vx=%.3f, vy=%.3f, vtheta=%.3f", 
                       vx_, vy_, vtheta_);
        }
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