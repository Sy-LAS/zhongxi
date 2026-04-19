#include "slam2d/map.h"
#include "utils/transformations.h"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace slam2d {

Map::Map(double resolution, int width, int height) 
    : resolution_(resolution), width_(width), height_(height), 
      origin_x_(-width_/2.0 * resolution_), origin_y_(-height_/2.0 * resolution_) {
    grid_map_.resize(width_ * height_);
}

void Map::updateGridMap(const LaserScan& scan, const Pose2D& robot_pose) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    // 使用激光雷达数据更新栅格地图
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        double range = scan.ranges[i];
        if (range < scan.range_min || range > scan.range_max) continue;
        
        // 计算障碍物点在全局坐标系中的位置
        double angle = scan.angles[i] + robot_pose.theta;
        double x = robot_pose.x + range * std::cos(angle);
        double y = robot_pose.y + range * std::sin(angle);
        
        // 更新终点的占用概率
        int idx = coordToIndex(x, y);
        if (idx >= 0 && idx < static_cast<int>(grid_map_.size())) {
            grid_map_[idx].occupancy_prob = std::min(0.9, grid_map_[idx].occupancy_prob + 0.3);
            grid_map_[idx].is_occupied = grid_map_[idx].occupancy_prob > 0.5;
            grid_map_[idx].observation_count++;
        }
        
        // 使用射线追踪算法更新无障碍区域
        double step_size = resolution_ * 0.5;
        int num_steps = static_cast<int>(range / step_size);
        
        for (int step = 0; step < num_steps; ++step) {
            double ratio = static_cast<double>(step) / num_steps;
            double ray_x = robot_pose.x + ratio * (x - robot_pose.x);
            double ray_y = robot_pose.y + ratio * (y - robot_pose.y);
            
            int ray_idx = coordToIndex(ray_x, ray_y);
            if (ray_idx >= 0 && ray_idx < static_cast<int>(grid_map_.size())) {
                grid_map_[ray_idx].occupancy_prob = std::max(0.1, grid_map_[ray_idx].occupancy_prob - 0.1);
                grid_map_[ray_idx].is_occupied = grid_map_[ray_idx].occupancy_prob > 0.5;
                grid_map_[ray_idx].observation_count++;
            }
        }
    }
}

void Map::updateFeatureMap(const std::vector<Observation>& observations, 
                          const Pose2D& robot_pose) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    for (const auto& obs : observations) {
        // 将观测点转换到全局坐标系
        Point2D global_point = utils::localToGlobal(robot_pose, obs.point);
        
        // 检查是否为已知特征点
        bool is_known_feature = false;
        for (auto& feature : feature_points_) {
            if (utils::distance(feature.position, global_point) < 2.0 * resolution_) {
                feature.observation_count++;
                feature.reliability = std::min(1.0, feature.reliability + 0.1);
                
                // 更新特征点位置（加权平均）
                double weight = 0.1;
                feature.position.x = (feature.position.x * (1.0 - weight) + global_point.x * weight);
                feature.position.y = (feature.position.y * (1.0 - weight) + global_point.y * weight);
                
                is_known_feature = true;
                break;
            }
        }
        
        // 如果是新特征点，添加到地图
        if (!is_known_feature) {
            FeaturePoint new_feature;
            new_feature.position = global_point;
            new_feature.id = static_cast<int>(feature_points_.size());
            new_feature.observation_count = 1;
            new_feature.reliability = 0.3;
            feature_points_.push_back(new_feature);
        }
    }
}

void Map::addLandmark(int landmark_id, const Point2D& global_position) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    landmarks_[landmark_id] = global_position;
}

bool Map::getLandmark(int landmark_id, Point2D& position) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto it = landmarks_.find(landmark_id);
    if (it != landmarks_.end()) {
        position = it->second;
        return true;
    }
    return false;
}

bool Map::isOccupied(double x, double y) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    int idx = coordToIndex(x, y);
    if (idx >= 0 && idx < static_cast<int>(grid_map_.size())) {
        return grid_map_[idx].is_occupied;
    }
    return false;  // 超出地图范围视为自由空间
}

double Map::getOccupancyProbability(double x, double y) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    int idx = coordToIndex(x, y);
    if (idx >= 0 && idx < static_cast<int>(grid_map_.size())) {
        return grid_map_[idx].occupancy_prob;
    }
    return 0.5;  // 超出地图范围返回0.5（未知）
}

int Map::coordToIndex(double x, double y) const {
    int col = static_cast<int>((x - origin_x_) / resolution_);
    int row = static_cast<int>((y - origin_y_) / resolution_);
    
    if (col < 0 || col >= width_ || row < 0 || row >= height_) {
        return -1;  // 超出地图范围
    }
    
    return row * width_ + col;
}

Point2D Map::indexToCoord(int idx) const {
    if (idx < 0 || idx >= width_ * height_) {
        return Point2D(0, 0);  // 返回原点作为错误标识
    }
    
    int row = idx / width_;
    int col = idx % width_;
    
    double x = origin_x_ + col * resolution_;
    double y = origin_y_ + row * resolution_;
    
    return Point2D(x, y);
}

void Map::clear() {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    for (auto& cell : grid_map_) {
        cell.occupancy_prob = 0.5;
        cell.is_occupied = false;
        cell.observation_count = 0;
    }
    
    feature_points_.clear();
    landmarks_.clear();
    trajectory_.clear();
}

void Map::addPoseToTrajectory(const Pose2D& pose) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    trajectory_.push_back(pose);
}

} // namespace slam2d