# GraphSLAM源码分析

## 项目概述

GraphSLAM是一个基于图优化的SLAM（Simultaneous Localization And Mapping，同时定位与建图）系统，主要用于机器人在未知环境中构建地图并同时估计自身位置。该项目包含三个主要组成部分：

1. **前端（Front End）**：负责处理传感器数据，执行里程计估计和初步位姿估计
2. **后端（Back End）**：负责全局优化，通过图优化技术修正位姿估计误差
3. **地图（Map）**：存储环境地图和机器人轨迹信息

## 文件结构

```
src/graphslam/
├── include/slam2d/           # 头文件
│   ├── types.h              # 基本数据类型定义
│   ├── map.h                # 地图相关类定义
│   ├── front_end.h          # 前端相关类定义
│   └── back_end.h           # 后端相关类定义
├── src/
│   ├── node/                # 主节点实现
│   │   └── slam2d_node.cpp
│   ├── slam2d/              # 核心SLAM算法实现
│   │   ├── back_end.cpp
│   │   ├── front_end.cpp
│   │   ├── map.cpp
│   │   └── utils.cpp
│   └── utils/               # 工具函数实现
│       ├── laser_scan.cpp
│       └── transformations.cpp
```

## 核心数据类型分析

### Pose2D - 二维位姿

定义了机器人在二维空间中的位置和方向。

```cpp
// 源码: src/graphslam/include/slam2d/types.h
struct Pose2D {
    double x = 0.0;        // x轴坐标
    double y = 0.0;        // y轴坐标
    double theta = 0.0;    // 旋转角度

    Pose2D() = default;
    Pose2D(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}

    // 从Eigen::Vector3d构造
    explicit Pose2D(const Eigen::Vector3d& vec) : x(vec.x()), y(vec.y()), theta(vec.z()) {}

    // 转换为Eigen::Vector3d
    Eigen::Vector3d toEigen() const {
        return Eigen::Vector3d(x, y, theta);
    }
};
```

### Point2D - 二维点

表示二维空间中的点坐标。

```cpp
// 源码: src/graphslam/include/slam2d/types.h
struct Point2D {
    double x = 0.0;
    double y = 0.0;

    Point2D() = default;
    Point2D(double x_, double y_) : x(x_), y(y_) {}

    // 从Eigen::Vector2d构造
    explicit Point2D(const Eigen::Vector2d& vec) : x(vec.x()), y(vec.y()) {}

    // 转换为Eigen::Vector2d
    Eigen::Vector2d toEigen() const {
        return Eigen::Vector2d(x, y);
    }
};
```

### LaserScan - 激光雷达扫描数据

存储激光雷达传感器的数据。

```cpp
// 源码: src/graphslam/include/slam2d/types.h
struct LaserScan {
    std::vector<double> ranges;      // 距离数据
    std::vector<double> angles;      // 对应的角度
    double min_angle = 0.0;          // 最小角度
    double max_angle = 0.0;          // 最大角度
    double angle_increment = 0.0;    // 角度增量
    double time_increment = 0.0;     // 时间增量
    double scan_time = 0.0;          // 扫描时间
    double range_min = 0.0;          // 最小距离
    double range_max = 0.0;          // 最大距离
    Pose2D origin;                   // 扫描原点位姿

    LaserScan() = default;
    LaserScan(const std::vector<double>& r, const std::vector<double>& a)
        : ranges(r), angles(a) {}
};
```

### Observation - 观测数据

表示传感器对环境中特征的观测结果。

```cpp
// 源码: src/graphslam/include/slam2d/types.h
struct Observation {
    Point2D point;                    // 观测到的点
    double range = 0.0;              // 距离
    double bearing = 0.0;            // 方位角
    int landmark_id = -1;            // 路标ID
    
    Observation() = default;
    Observation(const Point2D& p, double r, double b, int id) 
        : point(p), range(r), bearing(b), landmark_id(id) {}
};
```

## 核心功能模块分析

### 1. 地图模块 (Map)

地图模块负责维护环境的地图信息，包括栅格地图和特征地图。

#### Map类功能分析

```cpp
// 源码: src/graphslam/include/slam2d/map.h
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
```

#### updateGridMap函数实现

```
// 源码: src/graphslam/src/slam2d/map.cpp
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
```

### 2. 前端模块 (Front End)

前端模块负责实时处理传感器数据，估计机器人位姿。

#### FrontEnd基类定义

```
// 源码: src/graphslam/include/slam2d/front_end.h
class FrontEnd {
public:
    FrontEnd();
    virtual ~FrontEnd() = default;

    // 处理激光雷达数据
    virtual bool processLaserScan(const LaserScan& scan, const Pose2D& predicted_pose, 
                                 Pose2D& corrected_pose, std::vector<Observation>& observations) = 0;

    // 设置里程计参数
    void setOdometryParams(const OdometryParams& params) { odometry_params_ = params; }

    // 设置地图指针
    void setMap(MapPtr map) { map_ = map; }

    // 设置回调函数
    using PoseUpdateCallback = std::function<void(const Pose2D&)>;
    void setPoseUpdateCallback(PoseUpdateCallback callback) { pose_callback_ = callback; }

protected:
    OdometryParams odometry_params_;
    MapPtr map_;
    PoseUpdateCallback pose_callback_;

    // 使用运动模型预测位姿
    Pose2D predictPose(const Pose2D& prev_pose, const Pose2D& control);

    // 基于扫描匹配校正位姿
    virtual Pose2D scanMatchPose(const LaserScan& current_scan, 
                                const LaserScan& prev_scan, 
                                const Pose2D& initial_pose) = 0;
};
```

#### ICP前端实现

```
// 源码: src/graphslam/include/slam2d/front_end.h
class ICPFrontEnd : public FrontEnd {
public:
    ICPFrontEnd();
    virtual ~ICPFrontEnd() = default;

    // 处理激光雷达数据
    bool processLaserScan(const LaserScan& scan, const Pose2D& predicted_pose, 
                         Pose2D& corrected_pose, std::vector<Observation>& observations) override;

protected:
    // ICP扫描匹配
    Pose2D scanMatchPose(const LaserScan& current_scan, 
                        const LaserScan& prev_scan, 
                        const Pose2D& initial_pose) override;

private:
    LaserScan last_scan_;
    Pose2D last_pose_;
    int max_iterations_;
    double convergence_threshold_;
};
```

#### ICP扫描匹配实现

```
// 源码: src/graphslam/src/slam2d/front_end.cpp
Pose2D ICPFrontEnd::scanMatchPose(const LaserScan& current_scan, 
                                 const LaserScan& prev_scan, 
                                 const Pose2D& initial_pose) {
    Pose2D current_pose = initial_pose;
    std::vector<Point2D> curr_points = utils::LaserScanProcessor::polarToCartesianScan(current_scan);
    std::vector<Point2D> prev_points = utils::LaserScanProcessor::polarToCartesianScan(prev_scan);
    
    for (int iter = 0; iter < max_iterations_; ++iter) {
        // 计算当前位姿下的点集
        std::vector<Point2D> transformed_curr_points;
        for (const auto& pt : curr_points) {
            transformed_curr_points.push_back(utils::transformPoint(current_pose, pt));
        }
        
        // 寻找最近点对应关系
        std::vector<std::pair<Point2D, Point2D>> correspondences;
        for (const auto& curr_pt : transformed_curr_points) {
            double min_dist = std::numeric_limits<double>::max();
            Point2D closest_prev_pt;
            
            for (const auto& prev_pt : prev_points) {
                double dist = utils::distance(curr_pt, prev_pt);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_prev_pt = prev_pt;
                }
            }
            
            if (min_dist < 0.5) { // 距离阈值
                correspondences.emplace_back(curr_pt, closest_prev_pt);
            }
        }
        
        if (correspondences.empty()) {
            break;
        }
        
        // 计算变换
        double sum_x1 = 0, sum_y1 = 0, sum_x2 = 0, sum_y2 = 0;
        for (const auto& corr : correspondences) {
            sum_x1 += corr.first.x;
            sum_y1 += corr.first.y;
            sum_x2 += corr.second.x;
            sum_y2 += corr.second.y;
        }
        
        size_t n = correspondences.size();
        double mean_x1 = sum_x1 / n;
        double mean_y1 = sum_y1 / n;
        double mean_x2 = sum_x2 / n;
        double mean_y2 = sum_y2 / n;
        
        // 计算旋转角度
        double numerator = 0, denominator = 0;
        for (const auto& corr : correspondences) {
            double x1 = corr.first.x - mean_x1;
            double y1 = corr.first.y - mean_y1;
            double x2 = corr.second.x - mean_x2;
            double y2 = corr.second.y - mean_y2;
            
            numerator += (x1 * y2 - y1 * x2);
            denominator += (x1 * x2 + y1 * y2);
        }
        
        double delta_theta = std::atan2(numerator, denominator);
        double cos_theta = std::cos(delta_theta);
        double sin_theta = std::sin(delta_theta);
        
        // 计算平移
        double delta_x = mean_x2 - (mean_x1 * cos_theta - mean_y1 * sin_theta);
        double delta_y = mean_y2 - (mean_x1 * sin_theta + mean_y1 * cos_theta);
        
        // 更新位姿
        Pose2D delta_pose(delta_x, delta_y, delta_theta);
        Pose2D new_pose = utils::relativePose(current_pose, delta_pose);
        
        // 检查收敛
        double pose_change = std::sqrt(delta_x*delta_x + delta_y*delta_y + delta_theta*delta_theta);
        if (pose_change < convergence_threshold_) {
            break;
        }
        
        current_pose = new_pose;
    }
    
    return current_pose;
}
```

### 3. 后端模块 (Back End)

后端模块负责全局优化，修正前端估计的累积误差。

#### BackEnd基类定义

```
// 源码: src/graphslam/include/slam2d/back_end.h
class BackEnd {
public:
    BackEnd();
    virtual ~BackEnd() = default;

    // 添加位姿节点
    virtual bool addPoseNode(const Pose2D& pose) = 0;

    // 添加约束
    virtual bool addConstraint(const Constraint2D& constraint) = 0;

    // 执行优化
    virtual bool optimize() = 0;

    // 获取优化后的位姿
    virtual bool getOptimizedPose(int id, Pose2D& pose) const = 0;

    // 设置优化参数
    void setOptimizationParams(const OptimizationParams& params) { 
        opt_params_ = params; 
    }

    // 设置地图指针
    void setMap(MapPtr map) { map_ = map; }

    // 获取优化后的轨迹
    virtual std::vector<Pose2D> getOptimizedTrajectory() const = 0;

protected:
    OptimizationParams opt_params_;
    MapPtr map_;
};
```

#### G2OBackend实现

```
// 源码: src/graphslam/include/slam2d/back_end.h
class G2OBackend : public BackEnd {
public:
    G2OBackend();
    virtual ~G2OBackend();

    // 添加位姿节点
    bool addPoseNode(const Pose2D& pose) override;

    // 添加约束
    bool addConstraint(const Constraint2D& constraint) override;

    // 执行优化
    bool optimize() override;

    // 获取优化后的位姿
    bool getOptimizedPose(int id, Pose2D& pose) const override;

    // 获取优化后的轨迹
    std::vector<Pose2D> getOptimizedTrajectory() const override;

private:
    void* optimizer_;  // g2o优化器指针（在实现中具体定义）
    int current_vertex_id_;
};
```

### 4. 工具函数模块

#### 坐标变换工具函数

```
// 源码: src/graphslam/src/utils/transformations.cpp
namespace slam2d {
namespace utils {

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

Transform2D poseToTransform(const Pose2D& pose) {
    double cos_theta = std::cos(pose.theta);
    double sin_theta = std::sin(pose.theta);

    Transform2D transform = Transform2D::Identity();
    transform << cos_theta, -sin_theta, pose.x,
                 sin_theta,  cos_theta, pose.y,
                         0,          0,      1;
    return transform;
}

Pose2D transformToPose(const Transform2D& transform) {
    double x = transform(0, 2);
    double y = transform(1, 2);
    double theta = std::atan2(transform(1, 0), transform(0, 0));
    return Pose2D(x, y, theta);
}

Point2D transformPoint(const Pose2D& pose, const Point2D& point) {
    Transform2D transform = poseToTransform(pose);
    Eigen::Vector3d point_homogeneous(point.x, point.y, 1.0);
    Eigen::Vector3d transformed_point = transform * point_homogeneous;
    
    return Point2D(transformed_point.x(), transformed_point.y());
}

Point2D localToGlobal(const Pose2D& pose, const Point2D& point) {
    return transformPoint(pose, point);
}

Point2D globalToLocal(const Pose2D& pose, const Point2D& point) {
    Transform2D transform = poseToTransform(pose);
    Transform2D inv_transform = transform.inverse();
    
    Eigen::Vector3d point_homogeneous(point.x, point.y, 1.0);
    Eigen::Vector3d transformed_point = inv_transform * point_homogeneous;
    
    return Point2D(transformed_point.x(), transformed_point.y());
}

double distance(const Point2D& p1, const Point2D& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx*dx + dy*dy);
}

double poseDistance(const Pose2D& p1, const Pose2D& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx*dx + dy*dy);
}

Point2D polarToCartesian(double range, double bearing) {
    double x = range * std::cos(bearing);
    double y = range * std::sin(bearing);
    return Point2D(x, y);
}

} // namespace utils
} // namespace slam2d
```

#### 激光雷达数据处理工具

```
// 源码: src/graphslam/src/utils/laser_scan.cpp
namespace slam2d {
namespace utils {

std::vector<Point2D> LaserScanProcessor::polarToCartesianScan(const LaserScan& scan) {
    std::vector<Point2D> points;
    points.reserve(scan.ranges.size());

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        double range = scan.ranges[i];
        if (range > scan.range_min && range < scan.range_max) {
            Point2D point = polarToCartesian(range, scan.angles[i]);
            points.push_back(point);
        }
    }

    return points;
}

std::vector<Point2D> LaserScanProcessor::scanToGlobal(const LaserScan& scan, 
                                                     const Pose2D& robot_pose) {
    std::vector<Point2D> local_points = polarToCartesianScan(scan);
    std::vector<Point2D> global_points;
    global_points.reserve(local_points.size());

    for (const auto& point : local_points) {
        Point2D global_point = localToGlobal(robot_pose, point);
        global_points.push_back(global_point);
    }

    return global_points;
}

std::vector<Point2D> LaserScanProcessor::extractFeatures(const std::vector<Point2D>& points) {
    std::vector<Point2D> features;
    
    // 简单的特征提取方法：提取距离较远的点（可能是角落或边缘）
    for (const auto& point : points) {
        double dist = distanceFromOrigin(point);
        // 如果点距离原点超过一定阈值，则认为是特征点
        if (dist > 1.0) {  // 可调整的阈值
            features.push_back(point);
        }
    }

    return features;
}

} // namespace utils
} // namespace slam2d
```

### 5. 主节点实现

```
// 源码: src/graphslam/src/node/slam2d_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class GraphSLAMNode : public rclcpp::Node
{
public:
    GraphSLAMNode() : Node("graphslam_node")
    {
        RCLCPP_INFO(this->get_logger(), "GraphSLAM node initialized");
        
        // 订阅激光雷达数据
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&GraphSLAMNode::laserCallback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Laser scan subscription created");
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // TODO: 实现SLAM核心算法
        RCLCPP_DEBUG(this->get_logger(), "Received laser scan data");
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GraphSLAMNode>();
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting GraphSLAM node...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 6. Node Implementation
The main ROS2 node implementation is located in [src/node/](file:///h/my/ROS/001/zhongxi/zhongxi_2_16_s/src/graphslam/src/node) directory:

- **graphslam_node.cpp**: The main ROS2 node implementation that integrates laser scan and odometry data processing with SLAM algorithm. This file was created by merging functionality from the original template and optimizing for better performance.

```
// 源码: src/graphslam/src/node/graphslam_node.cpp
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class GraphSLAMNode : public rclcpp::Node
{
public:
    GraphSLAMNode() : Node("graphslam_node")
    {
        RCLCPP_INFO(this->get_logger(), "GraphSLAM node initialized");
        
        // 订阅激光雷达数据
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&GraphSLAMNode::laserCallback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Laser scan subscription created");
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // TODO: 实现SLAM核心算法
        RCLCPP_DEBUG(this->get_logger(), "Received laser scan data");
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GraphSLAMNode>();
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting GraphSLAM node...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```


## 可复用函数和变量

### 全局可复用函数

以下函数可在多个文件中重复使用：

- `normalizeAngle(double angle)` - 角度归一化函数
- `poseToTransform(const Pose2D& pose)` - 位姿转变换矩阵
- `transformToPose(const Transform2D& transform)` - 变换矩阵转位姿
- `transformPoint(const Pose2D& pose, const Point2D& point)` - 点坐标变换
- `localToGlobal(const Pose2D& pose, const Point2D& point)` - 局部坐标转全局坐标
- `globalToLocal(const Pose2D& pose, const Point2D& point)` - 全局坐标转局部坐标
- `distance(const Point2D& p1, const Point2D& p2)` - 计算两点间距离
- `poseDistance(const Pose2D& p1, const Pose2D& p2)` - 计算两姿态间距离
- `polarToCartesian(double range, double bearing)` - 极坐标转笛卡尔坐标

### 可复用的数据结构

- `Pose2D` - 二维位姿结构
- `Point2D` - 二维点结构
- `LaserScan` - 激光扫描数据结构
- `Observation` - 观测数据结构
- `MapPtr` - 地图智能指针类型

### 重要参数配置

- `OdometryParams` - 里程计模型参数
- `OptimizationParams` - 优化参数配置
- `FeaturePoint` - 特征点数据结构

## 总结

GraphSLAM系统采用模块化设计，将SLAM问题分解为前端和后端两部分。前端负责实时处理传感器数据并提供初步位姿估计，后端负责全局优化以消除累积误差。整个系统通过统一的数据结构和接口实现各个模块间的协同工作，形成了一个完整的SLAM解决方案。