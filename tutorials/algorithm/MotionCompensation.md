# 运动补偿算法（Motion Compensation Algorithm）详解

## 一、算法概述

运动补偿算法用于校正激光雷达扫描期间机器人运动导致的畸变。由于激光雷达逐点扫描,在扫描过程中机器人运动会导致点云畸变,需要通过运动补偿校正。

### 核心思想
根据机器人运动轨迹,按时间比例为每个激光点计算其在扫描开始时刻的位姿,从而校正点云。

---

## 二、核心函数

### 2.1 motionCompensation - 运动补偿

**文件位置**:`/home/chuil/Desktop/zhongxi/src/graphslam/src/utils/laser_scan.cpp`

```cpp
LaserScan LaserScanProcessor::motionCompensation(const LaserScan& scan, 
                                                 const Pose2D& start_pose,
                                                 const Pose2D& end_pose) {
    LaserScan compensated_scan = scan;
    
    // 计算扫描时间内的位姿变化
    Pose2D delta_pose = utils::relativePose(start_pose, end_pose);
    
    // 对每个激光点进行补偿
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        // 计算时间比例
        double time_ratio = static_cast<double>(i) / scan.ranges.size();
        
        // 线性插值位姿
        Pose2D current_pose;
        current_pose.x = start_pose.x + delta_pose.x * time_ratio;
        current_pose.y = start_pose.y + delta_pose.y * time_ratio;
        current_pose.theta = start_pose.theta + delta_pose.theta * time_ratio;
        current_pose.theta = utils::normalizeAngle(current_pose.theta);
        
        // 将激光点补偿到起始时刻
        Point2D point = utils::polarToCartesian(scan.ranges[i], scan.angles[i]);
        Point2D compensated_point = utils::globalToLocal(current_pose, 
                                                         utils::localToGlobal(start_pose, point));
        
        // 更新扫描数据
        double new_range = utils::distanceFromOrigin(compensated_point);
        double new_angle = std::atan2(compensated_point.y, compensated_point.x);
        
        compensated_scan.ranges[i] = new_range;
        compensated_scan.angles[i] = new_angle;
    }
    
    return compensated_scan;
}
```

---

## 三、算法原理

### 3.1 时间比例计算

```cpp
double time_ratio = static_cast<double>(i) / scan.ranges.size();
```

- 第0个点:`time_ratio = 0` (扫描开始)
- 最后一个点:`time_ratio = 1` (扫描结束)
- 中间点:线性插值

### 3.2 位姿插值

```cpp
current_pose.x = start_pose.x + delta_pose.x * time_ratio;
current_pose.y = start_pose.y + delta_pose.y * time_ratio;
current_pose.theta = start_pose.theta + delta_pose.theta * time_ratio;
```

**假设**:匀速运动模型

### 3.3 坐标变换

```cpp
// 点从当前位姿转换到起始位姿
Point2D compensated_point = utils::globalToLocal(current_pose, 
                                                  utils::localToGlobal(start_pose, point));
```

**变换链**:
```
局部坐标 → 全局坐标(起始位姿) → 局部坐标(当前位姿)
```

---

## 四、应用场景

### 4.1 高速运动

机器人运动速度 > 0.5m/s时,运动补偿尤为重要。

### 4.2 高精度建图

需要厘米级精度时,必须进行运动补偿。

---

## 五、参数说明

| 参数 | 说明 |
|------|------|
| `start_pose` | 扫描开始时的位姿 |
| `end_pose` | 扫描结束时的位姿 |
| `time_ratio` | 时间比例 [0, 1] |

---

## 六、总结

运动补偿算法:

1. **时间比例插值**:线性插值位姿
2. **坐标变换**:校正点云畸变
3. **适用场景**:高速运动、高精度建图
