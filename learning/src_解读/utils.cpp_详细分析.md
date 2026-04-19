---
tag: src/graphslam/src/slam2d/utils.cpp
---

# utils.cpp 详细分析

## 文件概述
utils.cpp是GraphSLAM系统中的工具函数集合，提供了数学计算、几何变换、数据处理等通用功能。该文件封装了多个独立的工具函数，为系统的其他模块提供基础支持，提高了代码的复用性和可维护性。

## 核心功能模块

### 1. 数学与几何变换工具
- **功能**: 提供常用的数学运算和几何变换功能
- **主要函数**: `transformPoint()`、`rotateVector()`、`composeTransforms()`
- **作用**: 实现坐标变换、向量运算等基础几何操作

```
// 示例代码段，实际实现可能有所不同
Point2D transformPoint(const Point2D& point, const Transform2D& transform) {
    // 应用旋转和平移变换
    double x_rotated = transform.rotation.cos() * point.x - 
                       transform.rotation.sin() * point.y;
    double y_rotated = transform.rotation.sin() * point.x + 
                       transform.rotation.cos() * point.y;
    
    Point2D result;
    result.x = x_rotated + transform.translation.x;
    result.y = y_rotated + transform.translation.y;
    
    return result;
}

Transform2D composeTransforms(const Transform2D& t1, const Transform2D& t2) {
    Transform2D result;
    // 计算复合旋转
    result.rotation = t1.rotation * t2.rotation;
    // 计算复合平移
    result.translation.x = t1.translation.x + 
                           t1.rotation.cos() * t2.translation.x - 
                           t1.rotation.sin() * t2.translation.y;
    result.translation.y = t1.translation.y + 
                           t1.rotation.sin() * t2.translation.x + 
                           t1.rotation.cos() * t2.translation.y;
    
    return result;
}
```

### 2. 激光雷达数据处理工具
- **功能**: 处理激光雷达数据的常用操作
- **主要函数**: `filterScan()`、`downsampleScan()`、`extractFeatures()`
- **作用**: 对激光雷达数据进行滤波、降采样、特征提取等预处理

```
// 示例代码段，实际实现可能有所不同
LaserScan filterScan(const LaserScan& scan, double min_range, double max_range) {
    LaserScan filtered_scan = scan;
    
    for (auto& range : filtered_scan.ranges) {
        if (range < min_range || range > max_range) {
            range = std::numeric_limits<double>::quiet_NaN();  // 设置为无效值
        }
    }
    
    return filtered_scan;
}

std::vector<Point2D> extractFeatures(const LaserScan& scan, const Pose2D& pose) {
    std::vector<Point2D> features;
    
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        if (!std::isnan(scan.ranges[i]) && scan.ranges[i] > 0) {
            double angle = scan.angle_min + i * scan.angle_increment;
            Point2D local_point;
            local_point.x = scan.ranges[i] * cos(angle);
            local_point.y = scan.ranges[i] * sin(angle);
            
            // 转换到世界坐标系
            Point2D world_point = transformPoint(local_point, pose);
            
            // 这里可以添加特征检测算法，例如角点检测
            if (isFeaturePoint(scan, i)) {
                features.push_back(world_point);
            }
        }
    }
    
    return features;
}
```

### 3. 数值优化工具
- **功能**: 提供数值优化相关的辅助函数
- **主要函数**: `gaussNewton()`、`solveLinearSystem()`、`computeJacobian()`
- **作用**: 为SLAM优化提供数值计算支持

```
// 示例代码段，实际实现可能有所不同
VectorXd gaussNewton(const std::function<VectorXd(const VectorXd&)>& residual_func,
                    const std::function<MatrixXd(const VectorXd&)>& jacobian_func,
                    const VectorXd& initial_guess,
                    int max_iterations = 100,
                    double tolerance = 1e-6) {
    VectorXd x = initial_guess;
    
    for (int i = 0; i < max_iterations; ++i) {
        VectorXd residuals = residual_func(x);
        MatrixXd J = jacobian_func(x);
        
        // 计算法方程: J^T * J * dx = -J^T * r
        MatrixXd JTJ = J.transpose() * J;
        VectorXd rhs = -J.transpose() * residuals;
        
        VectorXd dx = JTJ.ldlt().solve(rhs);
        
        x += dx;
        
        if (dx.norm() < tolerance) {
            break;
        }
    }
    
    return x;
}
```

### 4. 时间戳处理工具
- **功能**: 处理时间戳相关的操作
- **主要函数**: `getCurrentTimestamp()`、`timestampDiff()`、`syncTimestamps()`
- **作用**: 管理传感器数据的时间同步和时间差计算

```
// 示例代码段，实际实现可能有所不同
double getCurrentTimestamp() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

double timestampDiff(double ts1, double ts2) {
    return std::abs(ts1 - ts2);
}

std::vector<std::pair<LaserScan, ImuData>> syncTimestamps(
    const std::vector<LaserScan>& scans,
    const std::vector<ImuData>& imu_data,
    double max_sync_diff = 0.01) {
    std::vector<std::pair<LaserScan, ImuData>> synced_data;
    
    for (const auto& scan : scans) {
        // 寻找时间上最接近的IMU数据
        double min_diff = std::numeric_limits<double>::max();
        size_t best_idx = 0;
        
        for (size_t i = 0; i < imu_data.size(); ++i) {
            double diff = timestampDiff(scan.timestamp, imu_data[i].timestamp);
            if (diff < min_diff) {
                min_diff = diff;
                best_idx = i;
            }
        }
        
        if (min_diff <= max_sync_diff) {
            synced_data.emplace_back(scan, imu_data[best_idx]);
        }
    }
    
    return synced_data;
}
```

### 5. 数据可视化工具
- **功能**: 提供数据可视化的辅助函数
- **主要函数**: `plotTrajectory()`、`visualizeMap()`、`drawPoints()`
- **作用**: 将SLAM系统的中间结果可视化以便调试和展示

```
// 示例代码段，实际实现可能有所不同
void visualizeMap(const OccupancyGridMap& map) {
    cv::Mat img(map.getHeight(), map.getWidth(), CV_8UC1);
    
    for (int y = 0; y < map.getHeight(); ++y) {
        for (int x = 0; x < map.getWidth(); ++x) {
            double prob = map.getCellProbability(x, y);
            
            if (prob > 0.65) {
                img.at<uchar>(y, x) = 0;   // 黑色表示障碍物
            } else if (prob < 0.35) {
                img.at<uchar>(y, x) = 255; // 白色表示自由空间
            } else {
                img.at<uchar>(y, x) = 128; // 灰色表示未知区域
            }
        }
    }
    
    // 显示图像
    cv::imshow("Map Visualization", img);
    cv::waitKey(1);
}
```

### 6. 文件I/O工具
- **功能**: 提供数据读写功能
- **主要函数**: `readLaserScan()`、`writeTrajectory()`、`loadMap()`
- **作用**: 从文件读取数据或将结果保存到文件

```
// 示例代码段，实际实现可能有所不同
bool writeTrajectory(const std::vector<Pose2D>& trajectory, 
                    const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    for (const auto& pose : trajectory) {
        file << pose.x << " " << pose.y << " " << pose.theta << std::endl;
    }
    
    file.close();
    return true;
}
```

## 核心数据结构

### 1. Transform2D结构
- **功能**: 表示二维刚体变换
- **成员变量**: 旋转角度、平移向量

### 2. Point2D结构
- **功能**: 表示二维点
- **成员变量**: x坐标、y坐标

## 性能优化要点
- 尽量避免不必要的内存分配和复制
- 使用适当的数学库提高计算效率
- 对频繁调用的函数进行性能分析和优化

## 注意事项
- 工具函数应保持无状态，避免使用全局变量
- 需要处理边界条件和异常输入
- 保持接口简洁，职责单一