#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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
        
        // 发布机器人实际路径（用于可视化）
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/graphslam_path", 10);
        
        // 初始化路径消息
        robot_path_.header.frame_id = "map";

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
        
        // 更新并发布路径
        update_and_publish_path(msg);
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
    
    void update_and_publish_path(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        // 创建PoseStamped消息添加到路径中
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = odom_msg->header.stamp;
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose = odom_msg->pose.pose;
        
        // 只有当机器人移动了一定距离才添加新的路径点（避免路径点过多）
        if (robot_path_.poses.empty()) {
            robot_path_.poses.push_back(pose_stamped);
        } else {
            auto& last_pose = robot_path_.poses.back().pose;
            double dx = pose_stamped.pose.position.x - last_pose.position.x;
            double dy = pose_stamped.pose.position.y - last_pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            
            // 只有当移动超过0.05米时才添加新点
            if (dist > 0.05) {
                robot_path_.poses.push_back(pose_stamped);
                
                // 限制路径点数量，防止内存过大
                if (robot_path_.poses.size() > 10000) {
                    robot_path_.poses.erase(robot_path_.poses.begin());
                }
            }
        }
        
        // 更新时间戳并发布路径
        robot_path_.header.stamp = this->now();
        path_pub_->publish(robot_path_);
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
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // 存储最后的里程计位置
    geometry_msgs::msg::Pose last_odom_pose_;
    
    // 存储机器人路径
    nav_msgs::msg::Path robot_path_;
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