---
tag: src/graphslam/src/slam2d/map.cpp
---

# map.cpp 详细分析

## 文件概述
map.cpp实现了SLAM系统中的地图构建和管理功能。该文件主要负责将激光雷达数据转换为占用栅格地图，提供了地图更新、查询、保存等功能，是SLAM系统中环境表示的核心模块。

## 核心功能模块

### 1. 占用栅格地图初始化
- **功能**: 创建指定尺寸和分辨率的占用栅格地图
- **主要函数**: `initializeMap()`、`setMapProperties()`
- **作用**: 设置地图的基本属性，如尺寸、分辨率、原点位置等

```
// 示例代码段，实际实现可能有所不同
void OccupancyGridMap::initializeMap(double resolution, 
                                   int width, 
                                   int height, 
                                   double origin_x, 
                                   double origin_y) {
    resolution_ = resolution;
    width_ = width;
    height_ = height;
    origin_x_ = origin_x;
    origin_y_ = origin_y;
    
    // 初始化栅格值为未知状态
    map_data_.resize(width * height, 0.5);  // 0.5表示未知
}
```

### 2. 激光雷达数据栅格化
- **功能**: 将激光雷达的射线数据转换为栅格地图的占用概率
- **主要函数**: `updateWithLaserScan()`、`rayCasting()`
- **作用**: 根据激光雷达测量结果更新地图中对应栅格的占用概率

```
// 示例代码段，实际实现可能有所不同
void OccupancyGridMap::updateWithLaserScan(const LaserScan& scan, 
                                         const Pose2D& robot_pose) {
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        if (scan.ranges[i] >= scan.range_min && scan.ranges[i] <= scan.range_max) {
            // 计算激光束终点的世界坐标
            double angle = scan.angle_min + i * scan.angle_increment;
            double end_x = robot_pose.x + scan.ranges[i] * cos(robot_pose.theta + angle);
            double end_y = robot_pose.y + scan.ranges[i] * sin(robot_pose.theta + angle);
            
            // 获取起点（机器人位置）和终点的栅格坐标
            int start_grid_x, start_grid_y, end_grid_x, end_grid_y;
            worldToMap(robot_pose.x, robot_pose.y, start_grid_x, start_grid_y);
            worldToMap(end_x, end_y, end_grid_x, end_grid_y);
            
            // 执行光线投射算法
            rayCasting(start_grid_x, start_grid_y, end_grid_x, end_grid_y);
        }
    }
}
```

### 3. 光线投射算法
- **功能**: 实现经典的光线投射算法，更新从起点到终点路径上所有栅格的概率
- **主要函数**: `rayCasting()`、`bresenhamLine()`
- **作用**: 将激光雷达测量转化为栅格地图的占用概率更新

```
// 示例代码段，实际实现可能有所不同
void OccupancyGridMap::rayCasting(int start_x, int start_y, 
                                int end_x, int end_y) {
    // 使用Bresenham直线算法获取路径上的所有点
    auto line_points = bresenhamLine(start_x, start_y, end_x, end_y);
    
    // 更新路径上所有点的占用概率（除终点外）
    for (size_t i = 0; i < line_points.size() - 1; ++i) {
        int x = line_points[i].first;
        int y = line_points[i].second;
        
        if (isValidCell(x, y)) {
            updateCellProbability(x, y, false);  // 设置为自由空间
        }
    }
    
    // 更新终点的占用概率（终点通常认为是障碍物）
    if (isValidCell(end_x, end_y)) {
        updateCellProbability(end_x, end_y, true);  // 设置为占用空间
    }
}
```

### 4. 概率更新机制
- **功能**: 根据传感器模型更新栅格的占用概率
- **主要函数**: `updateCellProbability()`、`logOddsUpdate()`
- **作用**: 使用贝叶斯更新规则或对数几率更新规则修改栅格的占用概率

```
// 示例代码段，实际实现可能有所不同
void OccupancyGridMap::updateCellProbability(int x, int y, bool occupied) {
    int idx = y * width_ + x;
    
    if (occupied) {
        // 增加占用概率
        double p_occ = probability_hit_;  // 激光击中时的概率更新值
        map_data_[idx] = p_occ + (1.0 - p_occ) * map_data_[idx];
    } else {
        // 减少占用概率
        double p_free = probability_miss_;  // 激光穿过时的概率更新值
        map_data_[idx] = map_data_[idx] * p_free / (p_free + (1.0 - p_free) * map_data_[idx]);
    }
}
```

### 5. 地图查询与操作
- **功能**: 提供地图数据的查询、插值、可视化等操作
- **主要函数**: `getCellProbability()`、`isOccupied()`、`isFree()`
- **作用**: 为其他模块提供地图信息的访问接口

```
// 示例代码段，实际实现可能有所不同
double OccupancyGridMap::getCellProbability(double world_x, double world_y) const {
    int map_x, map_y;
    if (worldToMap(world_x, world_y, map_x, map_y)) {
        return map_data_[map_y * width_ + map_x];
    }
    return 0.5;  // 未知区域返回0.5
}

bool OccupancyGridMap::isOccupied(double world_x, double world_y) const {
    double prob = getCellProbability(world_x, world_y);
    return prob > occupancy_threshold_;
}

bool OccupancyGridMap::isFree(double world_x, double world_y) const {
    double prob = getCellProbability(world_x, world_y);
    return prob < free_threshold_;
}
```

### 6. 地图保存与加载
- **功能**: 将地图数据保存到文件或从文件加载
- **主要函数**: `saveToFile()`、`loadFromFile()`
- **作用**: 实现地图的持久化存储

```
// 示例代码段，实际实现可能有所不同
bool OccupancyGridMap::saveToFile(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    // 写入地图元数据
    file << resolution_ << " " << width_ << " " << height_ << " "
         << origin_x_ << " " << origin_y_ << std::endl;
    
    // 写入地图数据
    for (int i = 0; i < height_; ++i) {
        for (int j = 0; j < width_; ++j) {
            file << map_data_[i * width_ + j] << " ";
        }
        file << std::endl;
    }
    
    file.close();
    return true;
}
```

## 核心数据结构

### 1. OccupancyGridMap类
- **功能**: 表示占用栅格地图
- **成员变量**: 分辨率、宽度、高度、原点位置、地图数据数组

### 2. Cell类
- **功能**: 表示单个栅格单元
- **成员变量**: 占用概率、更新次数等

## 性能优化要点
- 使用高效的内存布局以提高缓存命中率
- 实现增量更新避免重复计算
- 采用多分辨率地图以平衡精度和效率

## 注意事项
- 概率更新参数对地图质量有重要影响
- 需要在地图分辨率和计算效率之间取得平衡
- 要考虑传感器噪声对地图构建的影响