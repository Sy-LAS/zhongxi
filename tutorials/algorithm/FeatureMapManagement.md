# 特征地图管理算法（Feature Map Management Algorithm）详解

## 一、算法概述

特征地图管理算法用于维护环境中的显著特征点集合,通过数据关联、位置更新和可靠性评估,构建稀疏的特征地图。

### 核心思想
将激光扫描中的显著点抽象为特征点,通过多次观测提高特征点位置的精度和可靠性。

### 算法流程
1. **观测转换**:将局部观测转换到全局坐标系
2. **数据关联**:匹配已知特征点
3. **位置更新**:加权平均更新特征点位置
4. **新特征添加**:添加未匹配的特征点

---

## 二、核心数据结构

### 2.1 FeaturePoint - 特征点结构

**文件位置**:`/home/chuil/Desktop/zhongxi/src/graphslam/include/slam2d/map.h`

```cpp
struct FeaturePoint {
    Point2D position;        // 位置
    int id;                  // 特征ID
    int observation_count;   // 观测次数
    double reliability;      // 可靠性 [0.0, 1.0]
    
    FeaturePoint() : id(-1), observation_count(0), reliability(0.0) {}
    FeaturePoint(const Point2D& pos, int feature_id) 
        : position(pos), id(feature_id), observation_count(0), reliability(0.0) {}
};
```

---

## 三、核心函数解析

### 3.1 updateFeatureMap - 特征地图更新

**文件位置**:`/home/chuil/Desktop/zhongxi/src/graphslam/src/slam2d/map.cpp` (第55-90行)

```cpp
void Map::updateFeatureMap(const std::vector<Observation>& observations, 
                          const Pose2D& robot_pose) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    for (const auto& obs : observations) {
        // 转换到全局坐标系
        Point2D global_point = utils::localToGlobal(robot_pose, obs.point);
        
        // 数据关联:检查是否为已知特征点
        bool is_known_feature = false;
        for (auto& feature : feature_points_) {
            if (utils::distance(feature.position, global_point) < 2.0 * resolution_) {
                // 匹配成功
                feature.observation_count++;
                feature.reliability = std::min(1.0, feature.reliability + 0.1);
                
                // 加权平均更新位置
                double weight = 0.1;
                feature.position.x = (feature.position.x * (1.0 - weight) + global_point.x * weight);
                feature.position.y = (feature.position.y * (1.0 - weight) + global_point.y * weight);
                
                is_known_feature = true;
                break;
            }
        }
        
        // 新特征点
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
```

---

## 四、算法原理

### 4.1 数据关联

**关联阈值**:
```cpp
if (utils::distance(feature.position, global_point) < 2.0 * resolution_)
```

- 阈值:`2.0 × resolution`
- 例如:分辨率0.05m,阈值为0.1m

**策略**:最近邻匹配(Nearest Neighbor)

### 4.2 位置更新

**加权平均**:
```cpp
double weight = 0.1;
feature.position.x = feature.position.x * 0.9 + global_point.x * 0.1;
feature.position.y = feature.position.y * 0.9 + global_point.y * 0.1;
```

**含义**:
- 保留90%的旧位置
- 融入10%的新观测
- 平滑噪声,提高精度

### 4.3 可靠性评估

**更新规则**:
```cpp
feature.reliability = std::min(1.0, feature.reliability + 0.1);
```

- 每次观测增加0.1
- 上限1.0
- 新特征初始值0.3

**可靠性解释**:

| 可靠性 | 观测次数 | 说明 |
|-------|---------|------|
| 0.3 | 1 | 新特征,不确定 |
| 0.5 | 3 | 中等可靠性 |
| 0.8 | 6 | 高可靠性 |
| 1.0 | 10+ | 非常可靠 |

---

## 五、路标管理

### 5.1 addLandmark - 添加路标

```cpp
void Map::addLandmark(int landmark_id, const Point2D& global_position) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    landmarks_[landmark_id] = global_position;
}
```

### 5.2 getLandmark - 查询路标

```cpp
bool Map::getLandmark(int landmark_id, Point2D& position) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto it = landmarks_.find(landmark_id);
    if (it != landmarks_.end()) {
        position = it->second;
        return true;
    }
    return false;
}
```

**数据结构**:`std::unordered_map<int, Point2D>`
- O(1)查询时间复杂度
- 适合大规模路标管理

---

## 六、参数调优

| 参数 | 默认值 | 说明 |
|------|--------|------|
| 关联阈值 | 2.0×resolution | 匹配距离阈值 |
| 更新权重 | 0.1 | 新观测权重 |
| 可靠性增量 | 0.1 | 每次观测增加量 |
| 初始可靠性 | 0.3 | 新特征初始值 |

---

## 七、总结

特征地图管理算法:

1. **数据关联**:最近邻匹配
2. **位置更新**:加权平均
3. **可靠性评估**:观测计数
