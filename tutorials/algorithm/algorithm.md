让我全面搜索项目中的所有核心算法实现。


现在我已经全面分析了项目，让我为您整理一份完整的核心算法清单：

## 📊 项目核心算法完整清单

根据对代码的全面分析，该项目包含以下核心算法：

---

### 🎯 **一、SLAM前端算法（FrontEnd）**

#### 1. **ICP（Iterative Closest Point）算法** ⭐
**文件位置**：`src/graphslam/src/slam2d/front_end.cpp`
- **类**：`ICPFrontEnd`
- **功能**：激光扫描帧间匹配，计算相邻位姿约束
- **核心方法**：
  - `scanMatchPose()` - ICP迭代优化主循环
  - `findClosestPoints()` - 暴力最近邻搜索（Brute-Force Nearest Neighbor）
  - `computeTransform()` - **SVD分解**求解最优变换
  - `composePoses()` - 位姿复合运算

**底层数学算法**：
- **SVD（奇异值分解）** - 使用Eigen库的`JacobiSVD`求解最优旋转矩阵
- **欧氏距离计算** - 点对点的几何距离
- **角度规范化** - `normalizeAngle()` 将角度约束到 [-π, π]

---

#### 2. **特征匹配算法**
**文件位置**：`src/graphslam/src/slam2d/front_end.cpp`
- **类**：`FeatureBasedFrontEnd`
- **功能**：基于特征点的扫描匹配（简化实现）
- **核心方法**：
  - `extractFeatures()` - 边缘特征检测（距离变化检测）
  - `scanMatchPose()` - 基于特征的位姿估计

---

#### 3. **差速驱动运动模型（Odometry Motion Model）**
**文件位置**：`src/graphslam/src/slam2d/front_end.cpp` (第13-41行)
- **函数**：`predictPose()`
- **功能**：基于里程计预测机器人位姿
- **特点**：
  - 差速驱动模型（旋转-平移-旋转分解）
  - 添加高斯噪声模型（`sampleNormalDistribution()`）
  - 误差传播模型（alpha1-alpha5参数）

---

### 🔧 **二、SLAM后端算法（BackEnd）**

#### 4. **图优化算法（Graph Optimization）** ⭐
**文件位置**：`src/graphslam/src/slam2d/back_end.cpp`
- **类**：`G2OBackend`
- **优化库**：**g2o (General Graph Optimization)**
- **核心算法**：

##### 4.1 **Levenberg-Marquardt (LM) 算法** 
- **实现**：`g2o::OptimizationAlgorithmLevenberg`
- **功能**：非线性最小二乘优化
- **参数**：
  - `lambda_init = 1e-3` - 初始阻尼因子
  - `lambda_factor = 10.0` - 阻尼调整因子
  - `max_iterations = 100` - 最大迭代次数
  - `epsilon = 1e-6` - 收敛阈值

##### 4.2 **Gauss-Newton 算法**
- **备选**：`g2o::OptimizationAlgorithmGaussNewton`
- **用途**：非线性优化的另一种选择

##### 4.3 **稀疏线性系统求解**
- **求解器**：`g2o::LinearSolverEigen`
- **矩阵类型**：稀疏矩阵（Sparse Matrix）
- **功能**：求解法方程 `H·Δx = -b`

##### 4.4 **图数据结构**
- **节点（Vertex）**：`g2o::VertexSE2` - 2D位姿节点
- **边（Edge）**：`g2o::EdgeSE2` - 位姿间约束边
- **优化器**：`g2o::SparseOptimizer` - 稀疏图优化器

---

### 🗺️ **三、建图算法（Mapping）**

#### 5. **占据栅格地图更新算法（Occupancy Grid Map）** ⭐
**文件位置**：`src/graphslam/src/slam2d/map.cpp`
- **类**：`Map`
- **函数**：`updateGridMap()`
- **核心算法**：

##### 5.1 **射线追踪算法（Ray Tracing / Ray Casting）**
- **功能**：更新激光束路径上的空闲区域
- **方法**：
  - 沿激光束方向逐步采样
  - 降低路径上栅格的占据概率
  - 步长：`step_size = resolution_ * 0.5`

##### 5.2 **贝叶斯概率更新（Bayesian Probability Update）**
- **障碍物点**：`occupancy_prob = min(0.9, prob + 0.3)` - 增加占据概率
- **空闲区域**：`occupancy_prob = max(0.1, prob - 0.1)` - 降低占据概率
- **阈值判断**：`is_occupied = (occupancy_prob > 0.5)`

---

#### 6. **特征地图管理算法**
**文件位置**：`src/graphslam/src/slam2d/map.cpp`
- **函数**：`updateFeatureMap()`
- **算法**：
  - **最近邻数据关联** - 距离阈值 `2.0 * resolution_`
  - **加权平均位置更新** - `weight = 0.1`
  - **可靠性评估** - `reliability = min(1.0, reliability + 0.1)`

---

#### 7. **路标管理算法（Landmark Management）**
- **函数**：`addLandmark()`, `getLandmark()`
- **数据结构**：哈希表 `std::unordered_map<int, Point2D>`

---

### 📐 **四、坐标变换与几何算法**

#### 8. **齐次坐标变换（Homogeneous Transformation）**
**文件位置**：`src/graphslam/src/utils/transformations.cpp`
- **核心函数**：
  - `poseToTransform()` - 构建3×3变换矩阵
  - `transformToPose()` - 从变换矩阵提取位姿
  - `transformPoint()` - 点的齐次变换
  - `localToGlobal()` - 局部坐标→全局坐标
  - `globalToLocal()` - 全局坐标→局部坐标

**变换矩阵形式**：
```
T = | cos(θ)  -sin(θ)  x |
    | sin(θ)   cos(θ)  y |
    |   0        0      1 |
```

---

#### 9. **极坐标-直角坐标转换**
- **函数**：`polarToCartesian()`
- **公式**：
  ```
  x = range × cos(bearing)
  y = range × sin(bearing)
  ```

---

#### 10. **欧氏距离计算**
- **函数**：`distance()`, `distanceFromOrigin()`, `poseDistance()`
- **公式**：`dist = sqrt((x1-x2)² + (y1-y2)²)`

---

#### 11. **相对位姿计算**
- **函数**：`relativePose()`
- **算法**：矩阵求逆 `T_relative = T_from⁻¹ × T_to`

---

### 🔍 **五、数据处理与特征提取**

#### 12. **激光扫描预处理**
**文件位置**：`src/graphslam/src/utils/laser_scan.cpp`
- **类**：`LaserScanProcessor`
- **函数**：
  - `filterScan()` - 过滤无效距离值
  - `polarToCartesianScan()` - 批量极坐标转直角坐标
  - `scanToGlobal()` - 激光点转换到全局坐标系

---

#### 13. **运动补偿算法（Motion Compensation）**
- **函数**：`motionCompensation()`
- **功能**：补偿激光扫描期间机器人的运动
- **方法**：按时间比例分配位姿变换

---

#### 14. **特征点提取**
- **函数**：`extractFeatures()`
- **算法**：基于距离阈值的特征筛选（`dist > 1.0m`）

---

### 🧭 **六、智能探索算法**

#### 15. **前沿探索算法（Frontier Exploration）** ⭐
**文件位置**：`src/graphslam/smart_explore_node.py`
- **类**：`SmartExploreNode`
- **核心函数**：`update_frontiers()`

**算法流程**：
1. **前沿点检测** - 寻找已知自由空间与未知区域的边界
2. **最近前沿选择** - 贪婪策略选择距离最近的前沿点
3. **导航执行** - 前往目标前沿点

**参数**：
- 未探索区域完成阈值：`5%`
- 前沿点更新频率：`1Hz`

---

#### 16. **避障算法（Obstacle Avoidance）**
- **函数**：`execute_avoidance()`
- **策略**：后退 + 转向组合动作
- **检测范围**：前方±30度

---

#### 17. **最短路径覆盖算法**
- **策略**：贪婪最近邻（Nearest Neighbor Greedy）
- **目标**：访问所有前沿点，覆盖未探索区域

---

### 📊 **七、回环检测算法**

#### 18. **基于距离的回环检测**
**文件位置**：`src/graphslam/include/slam2d/utils.h`
- **函数**：`loopDetection()`
- **算法**：
  - 位置距离阈值：`2.0m`
  - 角度差异阈值：`45° (π/4)`
  - 遍历候选位姿集合

---

### 📈 **八、轨迹处理算法**

#### 19. **滑动窗口轨迹平滑**
- **函数**：`smoothTrajectory()`
- **算法**：移动平均滤波（Moving Average Filter）
- **窗口大小**：默认3

---

#### 20. **路径长度计算**
- **函数**：`calculatePathLength()`
- **算法**：累加相邻位姿间的欧氏距离

---

### 🎨 **九、地图质量评估**

#### 21. **地图质量评分算法**
- **函数**：`evaluateMapQuality()`
- **评估指标**：
  - 探索覆盖率（权重60%）
  - 占用信息丰富度（权重40%）
- **输出**：归一化分数 [0, 1]

---

### 🔢 **十、数学工具算法**

#### 22. **正态分布采样**
- **函数**：`sampleNormalDistribution()`
- **实现**：C++ `<random>` 库的 `std::normal_distribution`
- **用途**：运动模型噪声注入

---

#### 23. **雅可比矩阵计算**
- **函数**：`rotationJacobian()`, `computeJacobian()`
- **用途**：优化和不确定性传播

---

#### 24. **四元数→欧拉角转换**
**文件位置**：`smart_explore_node.py` (第235-237行)
- **简化公式**（仅Z轴旋转）：
  ```python
  current_angle = atan2(2*(w*z + x*y), 1 - 2*(y² + z²))
  ```

---

## 📋 **算法汇总表**

| 序号 | 算法名称 | 类别 | 复杂度 | 关键性 |
|------|---------|------|--------|--------|
| 1 | **ICP + SVD** | 前端匹配 | O(N²) | ⭐⭐⭐⭐⭐ |
| 2 | **Levenberg-Marquardt** | 后端优化 | O(N³) | ⭐⭐⭐⭐⭐ |
| 3 | **射线追踪 + 贝叶斯更新** | 建图 | O(N) | ⭐⭐⭐⭐ |
| 4 | **前沿探索** | 导航 | O(N) | ⭐⭐⭐⭐ |
| 5 | **图优化（g2o）** | 后端 | O(N³) | ⭐⭐⭐⭐⭐ |
| 6 | **差速驱动模型** | 位姿预测 | O(1) | ⭐⭐⭐ |
| 7 | **回环检测** | 全局一致性 | O(N) | ⭐⭐⭐ |
| 8 | **坐标变换** | 几何计算 | O(1) | ⭐⭐⭐⭐ |

---

## 🏗️ **技术架构总结**

```
传感器数据 (激光雷达 + 里程计)
    ↓
[ICP算法 + SVD分解] → 帧间位姿约束
    ↓
[图优化构建，g2o] → 位姿节点 + 约束边
    ↓
[Levenberg-Marquardt优化] → 全局一致位姿
    ↓
[射线追踪 + 贝叶斯更新] → 占据栅格地图
    ↓
[前沿探索算法] → 自主导航决策
```

这是一个**完整的GraphSLAM系统**，核心依赖：
- **ICP算法**用于前端扫描匹配
- **g2o图优化**用于后端全局优化
- **贝叶斯概率**用于地图更新
- **前沿探索**用于自主导航