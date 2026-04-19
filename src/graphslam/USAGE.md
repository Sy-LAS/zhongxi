# GraphSLAM 与 Zhongxi机器人仿真系统使用说明

## 项目概述

本项目集成了Zhongxi机器人模型与GraphSLAM算法，可以在Gazebo仿真环境中进行SLAM实验。

## 目录结构

```
zhongxi/
├── src/
│   ├── graphslam/          # GraphSLAM算法包
│   │   ├── launch/         # 启动文件
│   │   ├── src/            # 源代码
│   │   │   ├── node/       # ROS2节点
│   │   │   ├── slam2d/     # 2D SLAM算法
│   │   │   └── utils/      # 工具函数
│   │   └── config/         # 配置文件
│   └── zhongxi_description/ # 机器人描述包
│       ├── urdf/           # 机器人模型定义
│       ├── meshes/         # 3D模型文件
│       ├── launch/         # 启动文件
│       └── config/         # 配置文件
```

## 启动方式

### 1. 单独查看机器人模型

```bash
cd /home/chuil/Desktop/zhongxi
colcon build --packages-select zhongxi_description
source install/setup.bash
ros2 launch zhongxi_description simple_view.launch.py
```

### 2. 在Gazebo中启动机器人

```bash
cd /home/chuil/Desktop/zhongxi
colcon build --packages-select zhongxi_description
source install/setup.bash
ros2 launch zhongxi_description gazebo_sim.launch.py
```

### 3. 启动带SLAM的完整仿真系统

```bash
cd /home/chuil/Desktop/zhongxi
colcon build
source install/setup.bash
ros2 launch graphslam slam_with_robot.launch.py
```

## ROS2节点、话题、服务和动作说明

### Nodes（节点）

1. **Robot State Publisher** (`robot_state_publisher`)
   - 功能：发布机器人静态和动态坐标变换
   - 包：`robot_state_publisher`
   - 可执行文件：`robot_state_publisher`

2. **Joint State Publisher** (`joint_state_publisher`)
   - 功能：发布机器人关节状态
   - 包：`joint_state_publisher`
   - 可执行文件：`joint_state_publisher`

3. **Gazebo Client** (`gazebo`)
   - 功能：Gazebo仿真环境
   - 包：`gazebo_ros`
   - 可执行文件：`gzclient`

4. **Gazebo Server** (`gazebo`)
   - 功能：Gazebo仿真服务器
   - 包：`gazebo_ros`
   - 可执行文件：`gzserver`

5. **Spawn Entity** (`spawn_entity.py`)
   - 功能：将机器人模型加载到Gazebo中
   - 包：`gazebo_ros`
   - 可执行文件：`spawn_entity.py`

6. **GraphSLAM Node** (`graphslam_node`)
   - 功能：执行SLAM算法
   - 包：`graphslam`
   - 可执行文件：`graphslam_node`

7. **RViz2** (`rviz2`)
   - 功能：可视化工具
   - 包：`rviz2`
   - 可执行文件：`rviz2`

### Topics（话题）

1. `/scan` - 激光雷达扫描数据
   - 消息类型：`sensor_msgs/msg/LaserScan`
   - 发布者：Gazebo中的激光雷达插件
   - 订阅者：SLAM节点

2. `/tf` 和 `/tf_static` - 坐标变换
   - 消息类型：`tf2_msgs/msg/TFMessage`
   - 发布者：Robot State Publisher
   - 订阅者：RViz2, SLAM节点

3. `/robot_description` - 机器人描述
   - 消息类型：`std_msgs/msg/String`
   - 发布者：参数服务器
   - 订阅者：Robot State Publisher

4. `/clock` - 仿真时间
   - 消息类型：`rosgraph_msgs/msg/Clock`
   - 发布者：Gazebo
   - 订阅者：所有需要仿真时间的节点

5. `/map` - 地图
   - 消息类型：`nav_msgs/msg/OccupancyGrid`
   - 发布者：SLAM节点
   - 订阅者：RViz2

6. `/odom` - 里程计数据
   - 消息类型：`nav_msgs/msg/Odometry`
   - 发布者：机器人控制器或仿真器
   - 订阅者：SLAM节点

### Services（服务）

1. `/gazebo/reset_simulation` - 重置仿真
2. `/gazebo/pause_physics` - 暂停物理仿真
3. `/gazebo/unpause_physics` - 恢复物理仿真
4. `/gazebo/get_model_state` - 获取模型状态

### Actions（动作）

当前系统未使用动作（actions），但可以在后续开发中添加例如导航等动作。

## 系统特性

1. **完整的URDF机器人模型**：包含底盘、激光雷达和摄像头的3D模型
2. **Gazebo仿真支持**：可在物理仿真环境中测试算法
3. **实时SLAM**：使用g2o优化库进行图优化
4. **可视化界面**：通过RViz2实时查看机器人状态和地图
5. **模块化设计**：易于扩展和维护

## 故障排除

1. **找不到包错误**：
   - 确保已执行 `source install/setup.bash`
   
2. **模型未显示**：
   - 检查meshes目录下的STL文件是否存在
   - 确认URDF文件中的路径是否正确

3. **SLAM节点无法启动**：
   - 确保已安装g2o库
   - 检查CMakeLists.txt中的依赖项

4. **Gazebo启动失败**：
   - 确保已正确安装Gazebo
   - 检查系统图形驱动是否正常