#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

class GraphSlamNode : public rclcpp::Node
{
public:
    GraphSlamNode() : Node("graphslam_node")
    {
        // 初始化TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // 订阅激光雷达数据
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&GraphSlamNode::laser_callback, this, std::placeholders::_1));

        // 订阅里程计数据
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&GraphSlamNode::odom_callback, this, std::placeholders::_1));

        // 发布地图到odom的TF变换
        map_to_odom_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/map_odom", 10);

        RCLCPP_INFO(this->get_logger(), "GraphSLAM node initialized");
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 处理激光雷达数据并传入SLAM算法
        RCLCPP_DEBUG(this->get_logger(), "Received laser scan data with %d points", 
                    static_cast<int>(msg->ranges.size()));
        
        // TODO: 在这里调用SLAM算法的实际实现
        // 当前只是示例，实际应用中需要集成后端SLAM算法
        process_laser_data(msg);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 处理里程计数据
        RCLCPP_DEBUG(this->get_logger(), "Received odometry data for frame %s", 
                    msg->header.frame_id.c_str());
        
        // TODO: 将里程计数据集成到SLAM算法中
        process_odom_data(msg);
    }

    void process_laser_data(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 这里应该调用实际的SLAM算法
        // 目前只是发布一个简单的TF变换作为示例
        publish_tf_transform(msg->header.stamp, msg->header.frame_id);
    }

    void process_odom_data(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 处理里程计数据的回调
        // 可能需要存储里程计信息以供SLAM算法使用
        last_odom_pose_ = msg->pose.pose;
    }

    void publish_tf_transform(const builtin_interfaces::msg::Time& stamp, const std::string& frame_id)
    {
        geometry_msgs::msg::TransformStamped t;
        
        t.header.stamp = stamp;
        t.header.frame_id = "map";
        t.child_frame_id = frame_id;

        // 使用最后的里程计位置作为参考
        t.transform.translation.x = last_odom_pose_.position.x;
        t.transform.translation.y = last_odom_pose_.position.y;
        t.transform.translation.z = last_odom_pose_.position.z;
        
        t.transform.rotation = last_odom_pose_.orientation;

        // 发布变换
        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr map_to_odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // 存储最后的里程计位置
    geometry_msgs::msg::Pose last_odom_pose_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting GraphSLAM node...");
    rclcpp::spin(std::make_shared<GraphSlamNode>());
    RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down GraphSLAM node...");
    rclcpp::shutdown();
    return 0;
}