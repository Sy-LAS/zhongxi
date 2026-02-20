#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "CYdLidar.h"
#include <memory>
#include <chrono>
#include <cmath>
#include <limits>
#include <unistd.h>

using namespace ydlidar;

namespace sensor_interfaces
{
    class SensorNode : public rclcpp::Node
    {
    public:
        explicit SensorNode(const std::string & node_name) : rclcpp::Node(node_name) {}
    };

    class LidarDriver : public SensorNode
    {
    public:
        LidarDriver();
        ~LidarDriver();

    private:
        bool initialize_sensor();
        void publish_scan();

        CYdLidar* laser;
        
        // 参数
        std::string port_;
        std::string frame_id_;
        std::string ignore_array_;
        int baudrate_;
        int lidar_type_;
        int device_type_;
        int sample_rate_;
        int abnormal_check_count_;
        bool resolution_fixed_;
        bool reversion_;
        bool inverted_;
        bool auto_reconnect_;
        bool is_single_channel_;
        bool intensity_;
        bool support_motor_dtr_;
        double angle_max_;
        double angle_min_;
        double range_max_;
        double range_min_;
        double frequency_;
        bool invalid_range_is_inf_;
        
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
        rclcpp::TimerBase::SharedPtr scan_timer_;
        bool is_initialized_;
    };
}

namespace sensor_interfaces
{
    LidarDriver::LidarDriver()
        : SensorNode("lidar_driver"), laser(nullptr), scan_pub_(nullptr), is_initialized_(false)
    {
        // 初始化参数 - 参考官方驱动实现
        // 获取参数
        this->declare_parameter<std::string>("port", "/dev/ydlidar");
        this->declare_parameter<std::string>("frame_id", "laser_frame");
        this->declare_parameter<std::string>("ignore_array", "");
        this->declare_parameter<int>("baudrate", 51200);
        this->declare_parameter<int>("lidar_type", TYPE_TRIANGLE);
        this->declare_parameter<int>("device_type", YDLIDAR_TYPE_SERIAL);
        this->declare_parameter<int>("sample_rate", 3);  // X3 Pro单通道使用3k
        this->declare_parameter<int>("abnormal_check_count", 4);
        this->declare_parameter<bool>("resolution_fixed", true);  // 固定分辨率
        this->declare_parameter<bool>("reversion", false);
        this->declare_parameter<bool>("inverted", true);  // 倒置
        this->declare_parameter<bool>("auto_reconnect", true);
        this->declare_parameter<bool>("isSingleChannel", true);  // 官方参数名
        this->declare_parameter<bool>("intensity", false);
        this->declare_parameter<bool>("support_motor_dtr", false);  // 官方参数名
        this->declare_parameter<double>("angle_max", 180.0);  // 角度单位为度
        this->declare_parameter<double>("angle_min", -180.0);  // 角度单位为度
        this->declare_parameter<double>("range_max", 64.0);
        this->declare_parameter<double>("range_min", 0.01);
        this->declare_parameter<double>("frequency", 8.0);
        this->declare_parameter<bool>("invalid_range_is_inf", false);

        this->get_parameter("port", port_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("ignore_array", ignore_array_);
        this->get_parameter("baudrate", baudrate_);
        this->get_parameter("lidar_type", lidar_type_);
        this->get_parameter("device_type", device_type_);
        this->get_parameter("sample_rate", sample_rate_);
        this->get_parameter("abnormal_check_count", abnormal_check_count_);
        this->get_parameter("resolution_fixed", resolution_fixed_);
        this->get_parameter("reversion", reversion_);
        this->get_parameter("inverted", inverted_);
        this->get_parameter("auto_reconnect", auto_reconnect_);
        this->get_parameter("isSingleChannel", is_single_channel_);  // 官方参数名
        this->get_parameter("intensity", intensity_);
        this->get_parameter("support_motor_dtr", support_motor_dtr_);  // 官方参数名
        this->get_parameter("angle_max", angle_max_);
        this->get_parameter("angle_min", angle_min_);
        this->get_parameter("range_max", range_max_);
        this->get_parameter("range_min", range_min_);
        this->get_parameter("frequency", frequency_);
        this->get_parameter("invalid_range_is_inf", invalid_range_is_inf_);

        // 创建发布者
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

        // 初始化YDLidar X3 Pro
        laser = new CYdLidar();
        
        // 尝试初始化传感器
        if (initialize_sensor()) {
            is_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "YDLidar X3 Pro初始化成功");
        } else {
            RCLCPP_ERROR(this->get_logger(), "YDLidar X3 Pro初始化失败");
        }

        // 创建定时器，用于发布扫描数据
        scan_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / frequency_)),
            std::bind(&LidarDriver::publish_scan, this)
        );
    }

    LidarDriver::~LidarDriver()
    {
        if (laser) {
            laser->turnOff();
            laser->disconnecting();
            delete laser;
            laser = nullptr;
        }
        RCLCPP_INFO(this->get_logger(), "YDLidar X3 Pro Driver节点已关闭");
    }

    bool LidarDriver::initialize_sensor()
    {
        RCLCPP_INFO(this->get_logger(), "尝试连接到YDLidar X3 Pro，端口: %s", port_.c_str());
        
        // 检查串口是否存在
        if (access(port_.c_str(), F_OK) != 0) {
            RCLCPP_ERROR(this->get_logger(), "串口设备不存在: %s", port_.c_str());
            return false;
        }
        
        // 检查串口是否可读写
        if (access(port_.c_str(), R_OK | W_OK) != 0) {
            RCLCPP_ERROR(this->get_logger(), "串口设备权限不足，请检查权限设置: %s", port_.c_str());
            RCLCPP_ERROR(this->get_logger(), "请尝试运行: sudo chmod 666 %s", port_.c_str());
            return false;
        }
        
        // 断开任何现有连接
        laser->disconnecting();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        //////////////////////string property/////////////////
        /// lidar port
        laser->setlidaropt(LidarPropSerialPort, port_.c_str(), port_.size());
        /// ignore array
        laser->setlidaropt(LidarPropIgnoreArray, ignore_array_.c_str(), ignore_array_.size());

        //////////////////////int property/////////////////
        /// lidar baudrate
        laser->setlidaropt(LidarPropSerialBaudrate, &baudrate_, sizeof(int));
        /// tof lidar
        laser->setlidaropt(LidarPropLidarType, &lidar_type_, sizeof(int));
        /// device type
        laser->setlidaropt(LidarPropDeviceType, &device_type_, sizeof(int));
        /// sample rate
        laser->setlidaropt(LidarPropSampleRate, &sample_rate_, sizeof(int));
        /// abnormal count
        laser->setlidaropt(LidarPropAbnormalCheckCount, &abnormal_check_count_, sizeof(int));

        //////////////////////bool property/////////////////
        /// fixed angle resolution
        laser->setlidaropt(LidarPropFixedResolution, &resolution_fixed_, sizeof(bool));
        /// rotate 180
        laser->setlidaropt(LidarPropReversion, &reversion_, sizeof(bool));
        /// Counterclockwise
        laser->setlidaropt(LidarPropInverted, &inverted_, sizeof(bool));
        laser->setlidaropt(LidarPropAutoReconnect, &auto_reconnect_, sizeof(bool));
        /// one-way communication
        laser->setlidaropt(LidarPropSingleChannel, &is_single_channel_, sizeof(bool));
        /// intensity
        laser->setlidaropt(LidarPropIntenstiy, &intensity_, sizeof(bool));
        /// Motor DTR
        laser->setlidaropt(LidarPropSupportMotorDtrCtrl, &support_motor_dtr_, sizeof(bool));

        //////////////////////float property/////////////////
        /// unit: °
        float f_optvalue = static_cast<float>(angle_max_);
        laser->setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
        f_optvalue = static_cast<float>(angle_min_);
        laser->setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
        /// unit: m
        f_optvalue = static_cast<float>(range_max_);
        laser->setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
        f_optvalue = static_cast<float>(range_min_);
        laser->setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
        /// unit: Hz
        f_optvalue = static_cast<float>(frequency_);
        laser->setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

        // 初始化激光雷达
        bool ret = laser->initialize();
        
        if (ret) {
            RCLCPP_INFO(this->get_logger(), "YDLidar X3 Pro初始化成功，尝试启动扫描...");
            
            // 等待设备完成初始化
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            
            // 启动扫描
            ret = laser->turnOn();
            
            if (!ret) {
                RCLCPP_ERROR(this->get_logger(), "启动YDLidar X3 Pro扫描失败: %s", laser->DescribeError());
                
                // 关闭激光雷达
                laser->turnOff();
            } else {
                RCLCPP_INFO(this->get_logger(), "YDLidar X3 Pro扫描已启动");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "YDLidar X3 Pro初始化失败: %s", laser->DescribeError());
        }

        if (!ret) {
            RCLCPP_ERROR(this->get_logger(), "YDLidar X3 Pro初始化失败，健康状态检查返回: ffffffff");
            RCLCPP_ERROR(this->get_logger(), "这通常是由于以下原因之一：");
            RCLCPP_ERROR(this->get_logger(), "1. 激光雷达固件故障或需要更新");
            RCLCPP_ERROR(this->get_logger(), "2. 硬件连接不稳定");
            RCLCPP_ERROR(this->get_logger(), "3. 设备被其他程序占用");
            RCLCPP_ERROR(this->get_logger(), "4. 激光雷达电源供应不足");
            RCLCPP_ERROR(this->get_logger(), "");
            RCLCPP_ERROR(this->get_logger(), "建议解决方案：");
            RCLCPP_ERROR(this->get_logger(), "1. 重新插拔激光雷达设备");
            RCLCPP_ERROR(this->get_logger(), "2. 检查USB线缆质量");
            RCLCPP_ERROR(this->get_logger(), "3. 检查设备是否被其他程序占用: lsof /dev/ttyUSB0");
            RCLCPP_ERROR(this->get_logger(), "4. 确保使用正确的波特率（X3 Pro默认为115200）");
            RCLCPP_ERROR(this->get_logger(), "5. 重启激光雷达固件（如有相关工具）");
        }

        return ret;
    }

    void LidarDriver::publish_scan()
    {
        if (!scan_pub_) {
            return;
        }

        // 只有在传感器初始化成功时才发布数据
        if (laser && is_initialized_) {
            LaserScan scan;
            if (laser->doProcessSimple(scan)) {
                auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
                
                scan_msg->header.stamp = this->now();
                scan_msg->header.frame_id = frame_id_;
                
                // SDK返回的角度配置已经是弧度单位，不需要转换
                scan_msg->angle_min = scan.config.min_angle;  // 已经是弧度
                scan_msg->angle_max = scan.config.max_angle;  // 已经是弧度
                scan_msg->angle_increment = scan.config.angle_increment;  // 已经是弧度
                scan_msg->time_increment = scan.config.time_increment;
                scan_msg->scan_time = scan.config.scan_time;
                scan_msg->range_min = scan.config.min_range;
                scan_msg->range_max = scan.config.max_range;
                
                // 设置数据长度
                scan_msg->ranges.resize(scan.points.size());
                scan_msg->intensities.resize(scan.points.size());

                // 填充数据
                for (size_t i = 0; i < scan.points.size(); ++i) {
                    const auto& point = scan.points[i];
                    scan_msg->ranges[i] = point.range;
                    scan_msg->intensities[i] = point.intensity;
                }
                
                scan_pub_->publish(*scan_msg);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "未能从YDLidar X3 Pro获取扫描数据");
            }
        } 
        // 如果传感器未初始化成功，则不发布任何数据
        else {
            RCLCPP_WARN_ONCE(this->get_logger(), "YDLidar X3 Pro未初始化，暂停发布scan数据");
        }
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sensor_interfaces::LidarDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    // 确保清理资源
    node.reset();
    return 0;
}