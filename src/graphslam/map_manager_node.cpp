#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <vector>
#include <cmath>
#include <queue>
#include <algorithm>

class MapManagerNode : public rclcpp::Node
{
public:
    MapManagerNode() : Node("map_manager_node")
    {
        // 初始化参数
        this->declare_parameter<double>("resolution", 0.1);  // 地图分辨率
        this->declare_parameter<int>("grid_size", 10);       // 每个网格的大小（以像素为单位）
        this->declare_parameter<double>("explored_threshold", 0.1); // 探索阈值
        this->declare_parameter<double>("free_space_threshold", 0.2); // 自由空间阈值
        
        resolution_ = this->get_parameter("resolution").as_double();
        grid_size_ = this->get_parameter("grid_size").as_int();
        explored_threshold_ = this->get_parameter("explored_threshold").as_double();
        free_space_threshold_ = this->get_parameter("free_space_threshold").as_double();
        
        // 订阅地图话题
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 1, std::bind(&MapManagerNode::mapCallback, this, std::placeholders::_1));
        
        // 订阅激光雷达数据
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MapManagerNode::laserCallback, this, std::placeholders::_1));
        
        // 发布探索完成状态
        exploration_complete_pub_ = this->create_publisher<std_msgs::msg::Bool>("/exploration_complete", 1);
        
        // 初始化TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 初始化定时器，定期检查探索状态
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), std::bind(&MapManagerNode::checkExplorationStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "Map Manager Node started");
    }

private:
    // 订阅者和发布者
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr exploration_complete_pub_;
    
    // TF相关
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 地图数据
    nav_msgs::msg::OccupancyGrid current_map_;
    bool map_received_ = false;
    
    // 分区数据
    std::vector<std::vector<bool>> exploration_grid_;  // 标记已探索区域
    std::vector<std::vector<bool>> boundary_grid_;     // 标记边界区域
    std::vector<std::vector<bool>> free_space_grid_;   // 标记自由空间
    
    // 参数
    double resolution_;
    int grid_size_;
    double explored_threshold_;
    double free_space_threshold_;
    
    // 激光雷达回调 - 更新已探索区域
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!map_received_) return;
        
        // 获取机器人在地图中的位置
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                current_map_.header.frame_id, 
                msg->header.frame_id, 
                tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }
        
        // 将机器人的位置转换为地图坐标
        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;
        
        int robot_map_x = (robot_x - current_map_.info.origin.position.x) / current_map_.info.resolution;
        int robot_map_y = (robot_y - current_map_.info.origin.position.y) / current_map_.info.resolution;
        
        // 更新探索网格 - 将激光雷达扫描到的区域标记为已探索
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min) {
                double angle = msg->angle_min + i * msg->angle_increment;
                
                // 计算激光束终点的坐标
                double endpoint_x = robot_x + msg->ranges[i] * std::cos(transform.transform.rotation.z + angle);
                double endpoint_y = robot_y + msg->ranges[i] * std::sin(transform.transform.rotation.z + angle);
                
                // 转换为地图坐标
                int map_x = (endpoint_x - current_map_.info.origin.position.x) / current_map_.info.resolution;
                int map_y = (endpoint_y - current_map_.info.origin.position.y) / current_map_.info.resolution;
                
                // 检查边界
                if (map_x >= 0 && map_x < (int)current_map_.info.width && 
                    map_y >= 0 && map_y < (int)current_map_.info.height) {
                    
                    // 标记这条路径上的点为已探索
                    markPathAsExplored(robot_map_x, robot_map_y, map_x, map_y);
                }
            }
        }
    }
    
    // 标记从起点到终点路径上的点为已探索
    void markPathAsExplored(int start_x, int start_y, int end_x, int end_y)
    {
        // 使用Bresenham直线算法标记路径
        int dx = std::abs(end_x - start_x);
        int dy = std::abs(end_y - start_y);
        int sx = (start_x < end_x) ? 1 : -1;
        int sy = (start_y < end_y) ? 1 : -1;
        int err = dx - dy;
        
        int x = start_x;
        int y = start_y;
        
        while (true) {
            // 检查边界
            if (x >= 0 && x < (int)current_map_.info.width && 
                y >= 0 && y < (int)current_map_.info.height) {
                
                // 计算对应的探索网格坐标
                int grid_x = x / grid_size_;
                int grid_y = y / grid_size_;
                
                if (grid_x >= 0 && grid_x < (int)exploration_grid_.size() && 
                    grid_y >= 0 && grid_y < exploration_grid_[0].size()) {
                    exploration_grid_[grid_x][grid_y] = true;
                }
            }
            
            if (x == end_x && y == end_y) break;
            
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }
    }
    
    // 地图回调
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_map_ = *msg;
        map_received_ = true;
        
        // 初始化探索网格和边界网格
        int grid_width = (msg->info.width / grid_size_) + 1;
        int grid_height = (msg->info.height / grid_size_) + 1;
        
        exploration_grid_.resize(grid_width, std::vector<bool>(grid_height, false));
        boundary_grid_.resize(grid_width, std::vector<bool>(grid_height, false));
        free_space_grid_.resize(grid_width, std::vector<bool>(grid_height, false));
        
        RCLCPP_INFO(this->get_logger(), "Map received and grids initialized: %dx%d", grid_width, grid_height);
    }
    
    // 检查探索状态
    void checkExplorationStatus()
    {
        if (!map_received_) {
            return;
        }
        
        // 更新边界网格和自由空间网格
        updateBoundaryAndFreeSpaceGrids();
        
        // 计算未探索的自由空间区域
        int unexplored_free_cells = 0;
        int total_free_cells = 0;
        
        for (size_t i = 0; i < exploration_grid_.size(); ++i) {
            for (size_t j = 0; j < exploration_grid_[i].size(); ++j) {
                if (free_space_grid_[i][j]) {
                    total_free_cells++;
                    if (!exploration_grid_[i][j]) {
                        unexplored_free_cells++;
                    }
                }
            }
        }
        
        // 计算探索完成百分比
        double exploration_percentage = total_free_cells > 0 ? 
            (double)(total_free_cells - unexplored_free_cells) / total_free_cells * 100.0 : 0.0;
        
        RCLCPP_INFO(this->get_logger(), 
            "Exploration status: %.2f%% (%d/%d free cells explored)", 
            exploration_percentage, total_free_cells - unexplored_free_cells, total_free_cells);
        
        // 如果未探索的自由空间小于阈值，则认为探索完成
        bool exploration_complete = (unexplored_free_cells == 0);
        
        // 发布探索完成状态
        auto msg = std_msgs::msg::Bool();
        msg.data = exploration_complete;
        exploration_complete_pub_->publish(msg);
        
        if (exploration_complete) {
            RCLCPP_INFO(this->get_logger(), "Exploration completed! All reachable areas have been mapped.");
        }
    }
    
    // 更新边界和自由空间网格
    void updateBoundaryAndFreeSpaceGrids()
    {
        if (!map_received_) return;
        
        // 遍历整个地图，更新边界和自由空间网格
        for (size_t i = 0; i < exploration_grid_.size(); ++i) {
            for (size_t j = 0; j < exploration_grid_[i].size(); ++j) {
                // 检查对应的地图区域
                int start_x = i * grid_size_;
                int start_y = j * grid_size_;
                int end_x = std::min(start_x + grid_size_, (int)current_map_.info.width);
                int end_y = std::min(start_y + grid_size_, (int)current_map_.info.height);
                
                int occupied_cells = 0;
                int free_cells = 0;
                int unknown_cells = 0;
                
                // 统计这个网格中的单元格类型
                for (int x = start_x; x < end_x; ++x) {
                    for (int y = start_y; y < end_y; ++y) {
                        int idx = y * current_map_.info.width + x;
                        if (idx < (int)current_map_.data.size()) {
                            int8_t value = current_map_.data[idx];
                            
                            if (value == -1) {  // 未知
                                unknown_cells++;
                            } else if (value >= 0 && value <= 20) {  // 自由空间
                                free_cells++;
                            } else {  // 占据或障碍物
                                occupied_cells++;
                            }
                        }
                    }
                }
                
                // 标记自由空间网格
                free_space_grid_[i][j] = (free_cells > 0);
                
                // 检查是否为边界网格（靠近障碍物或未知区域）
                boundary_grid_[i][j] = (occupied_cells > 0 || unknown_cells > 0);
            }
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapManagerNode>());
    rclcpp::shutdown();
    return 0;
}