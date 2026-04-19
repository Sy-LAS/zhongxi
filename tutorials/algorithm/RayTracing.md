# 射线追踪算法（Ray Tracing Algorithm）详解

## 一、算法概述

射线追踪算法（Ray Tracing/Ray Casting）用于在激光雷达扫描过程中更新占据栅格地图的空闲区域。当激光束从机器人发射到障碍物时,路径上的所有栅格都应该被标记为"空闲"(未被占用),而终点处的栅格被标记为"占用"。

### 核心思想
沿激光束传播方向逐步采样,更新路径上每个栅格的占据概率,实现环境的概率建图。

### 算法流程
1. **计算障碍物点**:根据激光距离和角度计算障碍物在全局坐标系中的位置
2. **射线采样**:沿激光束方向以固定步长采样
3. **空闲区域更新**:降低路径上栅格的占据概率
4. **障碍物更新**:增加终点栅格的占据概率

---

## 二、核心数据结构定义

### 2.1 GridCell - 栅格单元结构

**文件位置**:`/home/chuil/Desktop/zhongxi/src/graphslam/include/slam2d/map.h`

```cpp
struct GridCell {
    double occupancy_prob;  // 占用概率 [0.0, 1.0]
    bool is_occupied;       // 是否被占用
    int observation_count;  // 观测次数
    
    GridCell() : occupancy_prob(0.5), is_occupied(false), observation_count(0) {}
};
```

**成员说明**:
- `occupancy_prob`:占据概率
  - `0.0`:绝对空闲
  - `0.5`:未知
  - `1.0`:绝对占用
- `is_occupied`:二值占用状态(`occupancy_prob > 0.5`)
- `observation_count`:该栅格被观测的次数

### 2.2 Map - 地图类

**文件位置**:`/home/chuil/Desktop/zhongxi/src/graphslam/include/slam2d/map.h`

```cpp
class Map {
public:
    Map(double resolution = 0.05, int width = 2000, int height = 2000);
    void updateGridMap(const LaserScan& scan, const Pose2D& robot_pose);
    
private:
    double resolution_;           // 地图分辨率(m/cell)
    int width_, height_;         // 地图尺寸(cells)
    double origin_x_, origin_y_; // 地图原点
    std::vector<GridCell> grid_map_;
};
```

---

## 三、核心函数详细解析

### 3.1 updateGridMap - 栅格地图更新

**文件位置**:`/home/chuil/Desktop/zhongxi/src/graphslam/src/slam2d/map.cpp` (第15-53行)

**功能**:使用激光扫描数据和射线追踪算法更新占据栅格地图

```cpp
void Map::updateGridMap(const LaserScan& scan, const Pose2D& robot_pose) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    // 遍历所有激光束
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        double range = scan.ranges[i];
        if (range < scan.range_min || range > scan.range_max) continue;
        
        // 步骤1:计算障碍物点在全局坐标系中的位置
        double angle = scan.angles[i] + robot_pose.theta;
        double x = robot_pose.x + range * std::cos(angle);
        double y = robot_pose.y + range * std::sin(angle);
        
        // 步骤2:更新终点的占用概率(障碍物)
        int idx = coordToIndex(x, y);
        if (idx >= 0 && idx < static_cast<int>(grid_map_.size())) {
            grid_map_[idx].occupancy_prob = std::min(0.9, grid_map_[idx].occupancy_prob + 0.3);
            grid_map_[idx].is_occupied = grid_map_[idx].occupancy_prob > 0.5;
            grid_map_[idx].observation_count++;
        }
        
        // 步骤3:使用射线追踪算法更新无障碍区域
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

**参数详解**:
- `scan`:激光扫描数据
- `robot_pose`:机器人当前位姿

---

## 四、算法原理深度解析

### 4.1 障碍物点计算

**源码**:
```cpp
double angle = scan.angles[i] + robot_pose.theta;
double x = robot_pose.x + range * std::cos(angle);
double y = robot_pose.y + range * std::sin(angle);
```

**数学推导**:

激光点在机器人局部坐标系中:
```
x_local = range × cos(scan_angle)
y_local = range × sin(scan_angle)
```

转换到全局坐标系:
```
x_global = x_robot + x_local × cos(θ_robot) - y_local × sin(θ_robot)
y_global = y_robot + x_local × sin(θ_robot) + y_local × cos(θ_robot)
```

简化后(因为scan_angle是相对机器人朝向的角度):
```
angle = scan.angles[i] + robot_pose.theta
x = robot_pose.x + range × cos(angle)
y = robot_pose.y + range × sin(angle)
```

### 4.2 占据概率更新

**障碍物点更新**:
```cpp
grid_map_[idx].occupancy_prob = std::min(0.9, grid_map_[idx].occupancy_prob + 0.3);
```

**原理**:
- 遇到障碍物,增加占据概率
- 增量:`+0.3`
- 上限:`0.9`(不是1.0,保留一定不确定性)

**空闲区域更新**:
```cpp
grid_map_[ray_idx].occupancy_prob = std::max(0.1, grid_map_[ray_idx].occupancy_prob - 0.1);
```

**原理**:
- 激光束穿过的区域,降低占据概率
- 减量:`-0.1`
- 下限:`0.1`(不是0.0,保留一定不确定性)

**为什么增量和减量不对称?**
- `+0.3` vs `-0.1`
- 增加障碍物置信度更快(激光打到障碍物是强证据)
- 减少障碍物置信度更慢(需要多次观测才能确认是空闲)

### 4.3 射线采样策略

**源码**:
```cpp
double step_size = resolution_ * 0.5;
int num_steps = static_cast<int>(range / step_size);

for (int step = 0; step < num_steps; ++step) {
    double ratio = static_cast<double>(step) / num_steps;
    double ray_x = robot_pose.x + ratio * (x - robot_pose.x);
    double ray_y = robot_pose.y + ratio * (y - robot_pose.y);
}
```

**步长选择**:
- `step_size = resolution_ × 0.5`
- 步长为分辨率的一半,确保不遗漏任何栅格
- 例如:分辨率0.05m,步长0.025m

**线性插值**:
```
ray_x = x_robot + ratio × (x_obstacle - x_robot)
ray_y = y_robot + ratio × (y_obstacle - y_robot)

其中 ratio = step / num_steps ∈ [0, 1]
```

**几何解释**:
```
机器人 (x_robot, y_robot)
    |
    |  step=0, ratio=0
    |
    |  step=1, ratio=0.25
    |
    |  step=2, ratio=0.5
    |
    |  step=3, ratio=0.75
    |
    ↓
障碍物 (x_obstacle, y_obstacle), ratio=1
```

---

## 五、坐标转换详解

### 5.1 coordToIndex - 坐标转栅格索引

**文件位置**:`/home/chuil/Desktop/zhongxi/src/graphslam/src/slam2d/map.cpp` (第125-134行)

```cpp
int Map::coordToIndex(double x, double y) const {
    int col = static_cast<int>((x - origin_x_) / resolution_);
    int row = static_cast<int>((y - origin_y_) / resolution_);
    
    if (col < 0 || col >= width_ || row < 0 || row >= height_) {
        return -1;  // 超出地图范围
    }
    
    return row * width_ + col;
}
```

**计算公式**:
```
col = (x - origin_x) / resolution
row = (y - origin_y) / resolution
index = row × width + col
```

**地图原点**:
```cpp
origin_x_ = -width_/2.0 × resolution_
origin_y_ = -height_/2.0 × resolution_
```

地图中心在全局坐标系的原点`(0, 0)`。

---

## 六、数据流与算法流程

### 6.1 完整更新流程

```
激光扫描数据 (ranges[], angles[])
    ↓
遍历每条激光束 i
    ↓
计算障碍物全局坐标 (x, y)
    ↓
更新障碍物栅格:
  occupancy_prob += 0.3 (上限0.9)
    ↓
射线追踪:
  step_size = resolution × 0.5
  num_steps = range / step_size
    ↓
遍历每个采样点:
  计算插值坐标 (ray_x, ray_y)
    ↓
更新空闲栅格:
  occupancy_prob -= 0.1 (下限0.1)
    ↓
更新所有激光束完成
```

### 6.2 可视化示例

```
场景:机器人面对一面墙

      墙壁 (障碍物)
    ███████████████████
    █                 ↑
    █                 │ range = 3m
    █                 │
    █                 
    ○ ← 机器人
    
射线追踪过程:

    ○ ---·---·---·---·---·---·---·--- X
         ↓   ↓   ↓   ↓   ↓   ↓   ↓   ↓
       空闲 空闲 空闲 空闲 空闲 空闲 空闲 障碍物
       
    每个"·"代表一个采样点
    前7个点:occupancy_prob -= 0.1
    最后1个点:occupancy_prob += 0.3
```

---

## 七、贝叶斯概率更新理论

### 7.1 对数几率表示

虽然本项目使用直接的占据概率更新,但更严谨的方法是使用对数几率(Log-Odds):

```
L(m) = log(p(m) / (1 - p(m)))

更新公式:
L(m|z) = L(m) + L(z|m)

其中:
  L(z|m):传感器的逆模型
  障碍物:L(z|occupied) = log(0.9/0.1) ≈ 2.2
  空闲:L(z|free) = log(0.1/0.9) ≈ -2.2
```

### 7.2 本项目的简化方法

```cpp
// 障碍物
occupancy_prob = min(0.9, prob + 0.3)

// 空闲
occupancy_prob = max(0.1, prob - 0.1)
```

这是一种启发式方法,计算简单但数学严谨性不如对数几率。

---

## 八、参数调优指南

### 8.1 关键参数

| 参数 | 位置 | 默认值 | 说明 |
|------|------|--------|------|
| `step_size` | updateGridMap | resolution×0.5 | 射线采样步长 |
| 障碍物增量 | updateGridMap | +0.3 | 占据概率增加量 |
| 空闲减量 | updateGridMap | -0.1 | 占据概率减少量 |
| 上限 | updateGridMap | 0.9 | 最大占据概率 |
| 下限 | updateGridMap | 0.1 | 最小占据概率 |

### 8.2 不同场景配置

**高精度地图**:
```cpp
step_size = resolution * 0.3;  // 更密采样
障碍物增量 = +0.4;
空闲减量 = -0.15;
```

**快速建图**:
```cpp
step_size = resolution * 1.0;  // 稀疏采样
障碍物增量 = +0.2;
空闲减量 = -0.05;
```

---

## 九、常见问题与解决方案

### 9.1 地图出现孔洞

**原因**:步长过大,遗漏某些栅格

**解决**:减小`step_size`到`resolution * 0.3`

### 9.2 更新速度慢

**原因**:激光点数过多,射线步长过小

**解决**:
1. 增大`step_size`到`resolution * 0.8`
2. 限制激光点数量(降采样)

---

## 十、总结

射线追踪算法是占据栅格地图更新的核心:

1. **线性插值采样**:沿激光束方向均匀采样
2. **概率更新**:障碍物+0.3,空闲-0.1
3. **边界保护**:概率限制在[0.1, 0.9]
