#ifndef MAP_H
#define MAP_H

#include <vector>
#include <unordered_map>
#include <memory>
#include <mutex>
#include "types.h"

namespace slam2d {

// 栅格地图单元
struct GridCell {
    double occupancy_prob;  // 占用概率 [0.0, 1.0]
    bool is_occupied;       // 是否被占用
    int observation_count;  // 观测次数
    
    GridCell() : occupancy_prob(0.5), is_occupied(false), observation_count(0) {}
};

// 特征地图点
struct FeaturePoint {
    Point2D position;              // 位置
    int id;                        // 特征ID
    int observation_count;         // 观测次数
    double reliability;            // 可靠性
    
    FeaturePoint() : id(-1), observation_count(0), reliability(0.0) {}
    FeaturePoint(const Point2D& pos, int feature_id) 
        : position(pos), id(feature_id), observation_count(0), reliability(0.0) {}
};

// 地图类
class Map {
public:
    Map(double resolution = 0.05, int width = 2000, int height = 2000);
    ~Map() = default;

    // 更新栅格地图
    void updateGridMap(const LaserScan& scan, const Pose2D& robot_pose);
    
    // 更新特征地图
    void updateFeatureMap(const std::vector<Observation>& observations, 
                         const Pose2D& robot_pose);
    
    // 添加路标点
    void addLandmark(int landmark_id, const Point2D& global_position);
    
    // 获取路标点
    bool getLandmark(int landmark_id, Point2D& position) const;
    
    // 查询栅格占用状态
    bool isOccupied(double x, double y) const;
    
    // 查询栅格占用概率
    double getOccupancyProbability(double x, double y) const;
    
    // 转换坐标到栅格索引
    int coordToIndex(double x, double y) const;
    
    // 转换栅格索引到坐标
    Point2D indexToCoord(int idx) const;
    
    // 获取地图尺寸
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    double getResolution() const { return resolution_; }
    
    // 清空地图
    void clear();
    
    // 获取所有特征点
    const std::vector<FeaturePoint>& getFeaturePoints() const { return feature_points_; }
    
    // 获取所有路标点
    const std::unordered_map<int, Point2D>& getLandmarks() const { return landmarks_; }
    
    // 设置机器人轨迹
    void addPoseToTrajectory(const Pose2D& pose);
    
    // 获取机器人轨迹
    const std::vector<Pose2D>& getTrajectory() const { return trajectory_; }

private:
    double resolution_;                    // 地图分辨率(m/cell)
    int width_, height_;                  // 地图尺寸(cells)
    double origin_x_, origin_y_;          // 地图原点在全局坐标系中的位置
    
    std::vector<GridCell> grid_map_;      // 栅格地图数据
    std::vector<FeaturePoint> feature_points_;  // 特征点地图
    std::unordered_map<int, Point2D> landmarks_; // 路标点
    std::vector<Pose2D> trajectory_;      // 机器人轨迹
    
    mutable std::mutex map_mutex_;        // 地图访问互斥锁
};

using MapPtr = std::shared_ptr<Map>;

} // namespace slam2d

#endif // MAP_H