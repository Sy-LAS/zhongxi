# 滑动窗口轨迹平滑（Sliding Window Trajectory Smoothing）详解

## 一、算法概述

滑动窗口轨迹平滑算法用于消除机器人轨迹中的高频噪声,通过移动平均滤波(Moving Average Filter)对位姿序列进行平滑处理,得到更平滑的轨迹。

### 核心思想
对轨迹中的每个位姿,取其前后相邻位姿的平均值作为平滑后的位姿,窗口大小决定平滑程度。

### 算法流程
1. **窗口定义**:为每个位姿定义平滑窗口
2. **平均计算**:计算窗口内所有位姿的平均值
3. **角度处理**:特殊处理角度(规范化)
4. **轨迹重建**:用平滑后的位姿替换原位姿

---

## 二、核心函数

### 2.1 smoothTrajectory - 轨迹平滑

**文件位置**:`/home/chuil/Desktop/zhongxi/src/graphslam/src/slam2d/utils.cpp` (第127-158行)

```cpp
std::vector<Pose2D> SLAMUtils::smoothTrajectory(const std::vector<Pose2D>& trajectory, 
                                               int window_size) {
    if (trajectory.empty() || window_size <= 1) {
        return trajectory;
    }
    
    std::vector<Pose2D> smoothed_trajectory = trajectory;
    
    // 遍历轨迹中的每个位姿
    for (size_t i = 0; i < trajectory.size(); ++i) {
        // 计算窗口边界
        int start_idx = std::max(0, static_cast<int>(i) - window_size/2);
        int end_idx = std::min(static_cast<int>(trajectory.size()-1), 
                               static_cast<int>(i) + window_size/2);
        
        // 累加窗口内的位姿
        double sum_x = 0.0, sum_y = 0.0, sum_theta = 0.0;
        int count = 0;
        
        for (int j = start_idx; j <= end_idx; ++j) {
            sum_x += trajectory[j].x;
            sum_y += trajectory[j].y;
            sum_theta += trajectory[j].theta;
            count++;
        }
        
        // 计算平均值
        if (count > 0) {
            smoothed_trajectory[i].x = sum_x / count;
            smoothed_trajectory[i].y = sum_y / count;
            smoothed_trajectory[i].theta = utils::normalizeAngle(sum_theta / count);
        }
    }
    
    return smoothed_trajectory;
}
```

**参数详解**:
- `trajectory`:原始轨迹序列(输入)
- `window_size`:平滑窗口大小(输入),默认3
- **返回值**:平滑后的轨迹

---

## 三、算法原理

### 3.1 滑动窗口机制

**窗口计算**:
```cpp
int start_idx = std::max(0, static_cast<int>(i) - window_size/2);
int end_idx = std::min(static_cast<int>(trajectory.size()-1), 
                       static_cast<int>(i) + window_size/2);
```

**示例**(window_size=3):

```
轨迹索引:  0  1  2  3  4  5  6
          |__|  ← i=0, window=[0,1]
             |__|__|  ← i=1, window=[0,2]
                |__|__|  ← i=2, window=[1,3]
                   |__|__|  ← i=3, window=[2,4]
                      |__|__|  ← i=4, window=[3,5]
                         |__|__|  ← i=5, window=[4,6]
                            |__|  ← i=6, window=[5,6]
```

**边界处理**:
- 起始位置:`start_idx = max(0, ...)`
- 结束位置:`end_idx = min(size-1, ...)`
- 确保窗口不越界

### 3.2 移动平均滤波

**X坐标平滑**:
```cpp
smoothed_trajectory[i].x = sum_x / count;
```

**公式**:
```
x_smooth[i] = (1/N) × Σ x[j],  j ∈ [i-k, i+k]

其中:
  N:窗口内位姿数量
  k:window_size/2
```

**Y坐标平滑**:同理

### 3.3 角度平滑

**特殊处理**:
```cpp
smoothed_trajectory[i].theta = utils::normalizeAngle(sum_theta / count);
```

**为什么需要规范化?**

角度的周期性:
```
示例:
  角度1: 170° (≈ 3.0 rad)
  角度2: -170° (≈ -3.0 rad)
  
简单平均: (170 + (-170)) / 2 = 0°
但实际应该接近 180° 或 -180°
```

**规范化函数**:`utils::normalizeAngle()`

```cpp
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
```

确保角度在 `[-π, π]` 范围内。

---

## 四、数学推导

### 4.1 低通滤波特性

滑动平均滤波器是一种**低通滤波器**:

**频率响应**:
```
H(ω) = (1/N) × Σ e^(-jωn),  n=0,...,N-1

特性:
  - 低频信号:通过(增益≈1)
  - 高频信号:衰减(增益<1)
  - 截止频率:fc = fs / N
```

**物理意义**:
- 低频:机器人的真实运动
- 高频:传感器噪声、抖动
- 滤波后:保留真实运动,去除噪声

### 4.2 窗口大小影响

| window_size | 平滑程度 | 响应速度 | 相位延迟 |
|-------------|---------|---------|---------|
| 3 | 轻度平滑 | 快 | 小 |
| 5 | 中度平滑 | 中 | 中 |
| 7 | 重度平滑 | 慢 | 大 |
| 11 | 极强平滑 | 很慢 | 很大 |

**选择原则**:
- 噪声大:增大窗口
- 实时性要求高:减小窗口
- 典型值:3-7

---

## 五、数据流

```
原始轨迹 (含噪声)
  Pose0: (0.0, 0.0, 0.0)
  Pose1: (0.1, 0.0, 0.0)
  Pose2: (0.2, 0.0, 0.0)
  ...
    ↓
smoothTrajectory(window_size=3)
    ↓
遍历每个位姿:
  ├─ 计算窗口边界
  ├─ 累加窗口内位姿
  ├─ 计算平均值
  └─ 角度规范化
    ↓
平滑轨迹 (噪声减少)
  Pose0: (0.05, 0.0, 0.0)
  Pose1: (0.1, 0.0, 0.0)
  Pose2: (0.2, 0.0, 0.0)
  ...
```

---

## 六、可视化示例

### 6.1 一维位置平滑

```
原始轨迹(x坐标):
  0.0 → 0.12 → 0.18 → 0.31 → 0.39 → 0.52
  (含噪声,抖动±0.02m)
  
平滑后(window=3):
  0.06 → 0.15 → 0.21 → 0.29 → 0.41 → 0.46
  (更平滑,噪声减少)
```

### 6.2 轨迹平滑前后对比

```
原始轨迹(带噪声):
    ●
   / \
  ●   ●
 /     \
●       ●
 \     /
  ●   ●
   \ /
    ●
    
平滑后:
      ●
     /
    ●
   /
  ●
 /
●
```

---

## 七、应用场景

### 7.1 轨迹可视化

**用途**:在RViz中显示平滑的机器人轨迹

```cpp
// 获取优化轨迹
std::vector<Pose2D> trajectory = backend.getOptimizedTrajectory();

// 平滑处理
std::vector<Pose2D> smooth_traj = SLAMUtils::smoothTrajectory(trajectory, 5);

// 发布可视化
publishTrajectory(smooth_traj);
```

### 7.2 路径规划

**用途**:为路径规划器提供平滑的参考轨迹

```cpp
// 平滑轨迹作为参考
auto reference_path = smoothTrajectory(raw_path, 7);
path_planner.setReference(reference_path);
```

### 7.3 速度估计

**用途**:通过平滑轨迹计算更准确的速度

```cpp
// 平滑后的轨迹
auto smooth_traj = smoothTrajectory(trajectory, 3);

// 计算速度
for (size_t i = 1; i < smooth_traj.size(); ++i) {
    double dx = smooth_traj[i].x - smooth_traj[i-1].x;
    double dy = smooth_traj[i].y - smooth_traj[i-1].y;
    double dt = 0.1;  // 采样时间
    double velocity = sqrt(dx*dx + dy*dy) / dt;
}
```

---

## 八、参数调优

### 8.1 不同场景配置

**轻微噪声**:
```cpp
window_size = 3;
```

**中等噪声**:
```cpp
window_size = 5;
```

**严重噪声**:
```cpp
window_size = 7;
```

### 8.2 权衡考虑

| 考虑因素 | 小窗口 | 大窗口 |
|---------|--------|--------|
| 平滑效果 | 差 | 好 |
| 计算量 | 小 | 大 |
| 实时性 | 好 | 差 |
| 细节保留 | 好 | 差 |
| 相位延迟 | 小 | 大 |

---

## 九、常见问题与解决方案

### 9.1 过度平滑

**现象**:轨迹丢失重要细节,转弯处不自然

**原因**:窗口过大

**解决**:
1. 减小`window_size`
2. 使用自适应窗口(噪声大时增大,噪声小时减小)

### 9.2 边界效应

**现象**:轨迹起始和结束位置平滑效果差

**原因**:边界处窗口不完整

**解决**:
1. 使用镜像填充边界
2. 减小边界处的窗口大小

### 9.3 角度跳变

**现象**:平滑后角度出现跳变

**原因**:未正确规范化角度

**解决**:
```cpp
// 确保每次角度计算后都规范化
theta = utils::normalizeAngle(theta);
```

---

## 十、高级平滑方法

### 10.1 加权移动平均

```cpp
// 中心位姿权重更大
weights = [0.25, 0.5, 0.25];  // window=3
smoothed = Σ (weight[j] × pose[j])
```

### 10.2 卡尔曼滤波

```cpp
// 状态空间模型
x[k] = A × x[k-1] + w[k]
z[k] = H × x[k] + v[k]

// 预测-更新循环
predict();
update();
```

### 10.3 样条插值

```cpp
// B样条平滑
BSpline spline(trajectory);
smoothed = spline.evaluate(density=10);
```

---

## 十一、总结

滑动窗口轨迹平滑:

1. **移动平均**:简单有效的低通滤波
2. **角度规范化**:处理角度周期性
3. **边界保护**:窗口不越界
4. **参数可调**:window_size控制平滑程度

**优势**:
- 实现简单
- 计算量小
- 效果明显

**局限**:
- 相位延迟
- 固定窗口不够灵活
- 可能过度平滑

**适用场景**:
- 轨迹可视化
- 噪声过滤
- 路径规划预处理
