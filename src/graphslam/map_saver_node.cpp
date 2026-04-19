#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fstream>
#include <iostream>
#include <sys/stat.h>

class MapSaverNode : public rclcpp::Node
{
public:
    MapSaverNode() : Node("map_saver_node")
    {
        // 初始化参数
        this->declare_parameter<std::string>("map_save_path", "/home/chuil/Desktop/maps/");
        this->declare_parameter<std::string>("map_name", "explore_map");
        
        map_save_path_ = this->get_parameter("map_save_path").as_string();
        map_name_ = this->get_parameter("map_name").as_string();
        
        // 订阅地图话题
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 1, std::bind(&MapSaverNode::mapCallback, this, std::placeholders::_1));
        
        // 订阅探索完成状态
        exploration_complete_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/exploration_complete", 1, std::bind(&MapSaverNode::explorationCompleteCallback, this, std::placeholders::_1));
        
        // 初始化标志
        exploration_completed_ = false;
        map_received_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Map Saver Node started");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exploration_complete_sub_;
    
    std::string map_save_path_;
    std::string map_name_;
    
    bool exploration_completed_;
    bool map_received_;
    nav_msgs::msg::OccupancyGrid latest_map_;
    
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        latest_map_ = *msg;
        map_received_ = true;
        
        // 如果探索已完成且地图已接收，则保存地图
        if (exploration_completed_ && map_received_) {
            saveMap();
            exploration_completed_ = false;
            map_received_ = false;
        }
    }
    
    void explorationCompleteCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            exploration_completed_ = true;
            RCLCPP_INFO(this->get_logger(), "Exploration completed signal received. Preparing to save map...");
            
            // 如果地图已接收，则立即保存
            if (map_received_) {
                saveMap();
                exploration_completed_ = false;
                map_received_ = false;
            }
        }
    }
    
    void saveMap()
    {
        // 确保目录存在
        struct stat info;
        if (stat(map_save_path_.c_str(), &info) != 0) {
            // 目录不存在，创建它
            std::string cmd = "mkdir -p " + map_save_path_;
            system(cmd.c_str());
        }
        
        // 创建文件名
        std::string map_filename = map_save_path_ + map_name_ + ".pgm";
        std::string yaml_filename = map_save_path_ + map_name_ + ".yaml";
        
        // 保存PGM图像
        std::ofstream pgm_file(map_filename, std::ios::binary | std::ios::out);
        if (!pgm_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open file for writing: %s", map_filename.c_str());
            return;
        }
        
        // PGM header
        pgm_file << "P5\n";
        pgm_file << "# CREATOR: MapSaverNode\n";
        pgm_file << latest_map_.info.width << " " << latest_map_.info.height << "\n";
        pgm_file << "255\n";  // Max value
        
        // Write pixel data (convert from occupancy grid to image)
        for (int y = latest_map_.info.height - 1; y >= 0; y--) {
            for (unsigned int x = 0; x < latest_map_.info.width; x++) {
                unsigned int idx = x + y * latest_map_.info.width;
                int8_t value = latest_map_.data[idx];
                
                uint8_t pixel_value;
                if (value == -1) {  // Unknown
                    pixel_value = 205;  // Gray
                } else if (value == 0) {  // Free space
                    pixel_value = 255;  // White
                } else {  // Occupied
                    pixel_value = 0;    // Black
                }
                
                pgm_file.write(reinterpret_cast<char*>(&pixel_value), sizeof(uint8_t));
            }
        }
        
        pgm_file.close();
        
        // 保存YAML文件
        std::ofstream yaml_file(yaml_filename);
        if (!yaml_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open YAML file for writing: %s", yaml_filename.c_str());
            return;
        }
        
        yaml_file << "image: " << map_name_ << ".pgm\n";
        yaml_file << "resolution: " << latest_map_.info.resolution << "\n";
        yaml_file << "origin: [" << latest_map_.info.origin.position.x << ", "
                  << latest_map_.info.origin.position.y << ", "
                  << 0.0 << "]\n";
        yaml_file << "negate: 0\n";
        yaml_file << "occupied_thresh: 0.65\n";
        yaml_file << "free_thresh: 0.196\n";
        
        yaml_file.close();
        
        RCLCPP_INFO(this->get_logger(), "Map saved successfully to %s and %s", 
                   map_filename.c_str(), yaml_filename.c_str());
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapSaverNode>());
    rclcpp::shutdown();
    return 0;
}