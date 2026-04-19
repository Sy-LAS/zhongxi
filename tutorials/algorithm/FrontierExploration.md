# 前沿探索算法（Frontier Exploration Algorithm）详解

## 一、算法概述

前沿探索算法是自主导航的核心技术,用于在未知环境中自动发现并探索新的区域。前沿点(Frontier)是已知自由空间和未知区域的边界,探索这些边界可以最大化信息获取。

### 核心思想
通过识别地图上已知和未知区域的边界点,选择最近的前沿点作为目标,驱动机器人前往探索。

### 算法流程
1. **地图解析**:获取占据栅格地图数据
2. **前沿检测**:寻找自由空间与未知区域的边界
3. **目标选择**:选择距离最近的前沿点
4. **导航执行**:前往目标前沿点

---

## 二、核心类定义

### 2.1 SmartExploreNode 类

**文件位置**:`/home/chuil/Desktop/zhongxi/src/graphslam/smart_explore_node.py`

```python
class SmartExploreNode(Node):
    def __init__(self):
        super().__init__('smart_explore_node')
        
        # 发布速度命令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 订阅传感器数据
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # 探索状态
        self.frontier_points = []          # 前沿点列表
        self.target_frontier = None        # 目标前沿点
        self.unexplored_ratio = 0.0        # 未探索区域比例
```

---

## 三、核心函数详细解析

### 3.1 update_frontiers - 前沿点检测

**文件位置**:`/home/chuil/Desktop/zhongxi/src/graphslam/smart_explore_node.py` (第128-183行)

```python
def update_frontiers(self):
    """更新前沿点列表"""
    if not self.current_map:
        return
        
    # 获取地图参数
    resolution = self.current_map.info.resolution
    origin_x = self.current_map.info.origin.position.x
    origin_y = self.current_map.info.origin.position.y
    width = self.current_map.info.width
    height = self.current_map.info.height
    
    # 获取地图数据
    map_data = np.array(self.current_map.data).reshape((height, width))
    
    # 寻找前沿点
    frontiers = []
    for y in range(1, height-1):
        for x in range(1, width-1):
            if map_data[y][x] == 0:  # 已知自由空间
                # 检查周围8邻域
                neighbors = [
                    map_data[y-1][x-1], map_data[y-1][x], map_data[y-1][x+1],
                    map_data[y][x-1],                   map_data[y][x+1],
                    map_data[y+1][x-1], map_data[y+1][x], map_data[y+1][x+1]
                ]
                
                if -1 in neighbors:  # 周围有未知区域
                    world_x = origin_x + x * resolution
                    world_y = origin_y + y * resolution
                    frontiers.append((world_x, world_y))
    
    self.frontier_points = frontiers
    
    # 选择最近的前沿点
    if frontiers and self.robot_x is not None and self.robot_y is not None:
        distances = [math.sqrt((fx - self.robot_x)**2 + (fy - self.robot_y)**2) 
                    for fx, fy in frontiers]
        min_idx = distances.index(min(distances))
        self.target_frontier = frontiers[min_idx]
```

**参数说明**:
- 地图值含义:
  - `-1`:未知区域
  - `0`:自由空间
  - `100`:占用区域

**算法步骤**:

1. **地图数据重塑**:
   ```python
   map_data = np.array(self.current_map.data).reshape((height, width))
   ```
   - 将1D数组转换为2D矩阵

2. **8邻域检测**:
   ```python
   if map_data[y][x] == 0:  # 当前是自由空间
       neighbors = [...]     # 获取8个邻居
       if -1 in neighbors:   # 有未知邻居
           # 这是前沿点
   ```

3. **坐标转换**:
   ```python
   world_x = origin_x + x * resolution
   world_y = origin_y + y * resolution
   ```
   - 从栅格索引转换到世界坐标

4. **目标选择**:
   ```python
   distances = [计算到每个前沿点的距离]
   min_idx = 最小距离的索引
   self.target_frontier = 最近的前沿点
   ```

**策略**:贪婪最近邻(Greedy Nearest Neighbor)

---

### 3.2 execute_exploration - 探索执行

**文件位置**:`/home/chuil/Desktop/zhongxi/src/graphslam/smart_explore_node.py` (第217-275行)

```python
def execute_exploration(self, cmd_msg):
    """执行探索行为 - 前往最近的前沿点"""
    if self.target_frontier is None:
        # 原地旋转寻找
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = self.angular_speed
        return
    
    # 计算到目标的角度和距离
    target_x, target_y = self.target_frontier
    dx = target_x - self.robot_x
    dy = target_y - self.robot_y
    dist_to_target = math.sqrt(dx*dx + dy*dy)
    target_angle = math.atan2(dy, dx)
    
    # 获取机器人当前朝向
    if self.current_odom:
        quat = self.current_odom.pose.pose.orientation
        current_angle = math.atan2(2*(quat.w*quat.z + quat.x*quat.y), 
                                  1 - 2*(quat.y*quat.y + quat.z*quat.z))
    else:
        current_angle = 0.0
    
    # 计算角度差
    angle_diff = target_angle - current_angle
    while angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    while angle_diff < -math.pi:
        angle_diff += 2 * math.pi
    
    # 到达判断
    if dist_to_target < 0.5:
        self.has_reached_target = True
        self.update_frontiers()  # 寻找下一个前沿点
    
    # 运动控制
    if abs(angle_diff) > 0.2:  # 先转向
        cmd_msg.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
        cmd_msg.linear.x = 0.0
    else:  # 方向正确,前进
        cmd_msg.linear.x = self.linear_speed
        cmd_msg.angular.z = 0.0
```

**控制策略**:

| 条件 | 线速度 | 角速度 | 行为 |
|------|--------|--------|------|
| 角度差 > 0.2rad | 0 | ±0.5 | 转向 |
| 角度差 ≤ 0.2rad | 0.3 | 0 | 前进 |
| 距离 < 0.5m | - | - | 到达目标 |

---

## 四、探索完成判断

### 4.1 未探索区域比例计算

**文件位置**:`/home/chuil/Desktop/zhongxi/src/graphslam/smart_explore_node.py` (第96-120行)

```python
def map_callback(self, msg):
    """处理地图数据"""
    with self.map_mutex:
        self.current_map = msg
        
        # 计算未探索区域比例
        total_cells = len(msg.data)
        unexplored_cells = sum(1 for cell in msg.data if cell == -1)
        self.unexplored_ratio = unexplored_cells / total_cells if total_cells > 0 else 0
        
        # 探索完成判断
        if self.unexplored_ratio < 0.05:  # 小于5%
            self.exploration_active = False
```

**阈值**:5%未探索区域认为探索完成

---

## 五、数据流

```
占据栅格地图 (OccupancyGrid)
    ↓
map_callback()
    ↓
update_frontiers()
    ├─ 遍历地图栅格
    ├─ 8邻域检测
    └─ 前沿点列表
    ↓
选择最近前沿点
    ↓
execute_exploration()
    ├─ 计算角度差
    ├─ 转向控制
    └─ 前进控制
    ↓
/cmd_vel → 机器人运动
```

---

## 六、参数调优

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `linear_speed` | 0.3 m/s | 前进速度 |
| `angular_speed` | 0.5 rad/s | 转向速度 |
| 到达阈值 | 0.5 m | 目标到达距离 |
| 角度阈值 | 0.2 rad (11°) | 转向阈值 |
| 探索完成阈值 | 5% | 未探索区域比例 |

---

## 七、总结

前沿探索算法:

1. **8邻域检测**:识别前沿点
2. **贪婪策略**:选择最近前沿
3. **比例控制**:转向+前进
4. **自动终止**:未探索区域<5%
