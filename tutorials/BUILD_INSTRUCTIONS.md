# GraphSLAM 构建和运行说明

## 包信息
- **包名**: graphslam 
- **包类型**: ROS 2 ament_cmake 包
- **位置**: ~/Desktop/zhongxi/src/graphslam/

## 构建命令
```bash
cd ~/Desktop/zhongxi
colcon build --packages-select graphslam
```

## 运行命令
首先设置环境：
```bash
source ~/Desktop/zhongxi/install/setup.bash
```

然后运行节点：
```bash
# 运行SLAM主节点
ros2 run graphslam graphslam_node

# 运行可视化界面
ros2 run graphslam graphslam_g2o
```

## 常见问题
1. **错误**: `ignoring unknown package 'GraphSLAM'`
   - **解决**: 使用 `graphslam` 而不是 `GraphSLAM`

2. **找不到可执行文件**
   - **解决**: 确保先运行 `source ~/Desktop/zhongxi/install/setup.bash`

## 项目结构
```
~/Desktop/zhongxi/
├── src/
│   └── graphslam/          # 源代码目录
├── build/                  # 构建输出
├── install/                # 安装目录
└── log/                    # 日志文件
```