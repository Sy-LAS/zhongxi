# 传感器接口集成说明

## 项目概述

本项目实现了对YDLidar X3 Pro激光雷达和WHEELTEC C100摄像头的接口，用于在Jetson Orin Nano平台上进行SLAM。

## 传感器真实参数

### YDLidar X3 Pro
- 型号：X3 Pro
- 测距范围：0.01m - 8m
- 扫描频率：5-12Hz
- 角度范围：360°
- 角分辨率：0.33°
- 通讯方式：UART/TTL
- 波特率：230400

### WHEELTEC C100
- 型号：C100
- 分辨率：支持640x480, 1280x720
- 帧率：30fps
- 接口：USB 2.0
- 传感器：CMOS

## 当前实现状态

### 激光雷达驱动 (lidar_driver)
- **当前状态**：已集成官方YDLidar SDK
- **参数**：已按照真实X3 Pro规格配置
- **功能**：直接与真实硬件通信，不再使用模拟实现

### 摄像头驱动 (camera_driver)
- **当前状态**：OpenCV真实硬件实现
- **参数**：已按C100规格配置
- **功能**：直接从USB摄像头设备读取图像

## 硬件集成步骤

### 1. 激光雷达集成
YDLidar X3 Pro已经通过官方SDK集成：

1. YDLidar SDK已复制到`sensor_interfaces/sdk/`目录
2. CMakeLists.txt已更新以编译SDK源码
3. lidar_driver.cpp已更新以使用真实的YDLidar驱动接口

关键配置参数（针对X3 Pro）：
```cpp
baudrate = 230400;        // X3 Pro波特率
lidar_type = TYPE_TRIANGLE;  // 三角测距类型
isSingleChannel = false;  // X3 Pro是多通道，区别于X3
range_min = 0.01;         // 最小测距0.01m
range_max = 8.0;         // 最大测距25m
```

### 2. 摄像头集成
摄像头驱动使用OpenCV直接从USB摄像头设备读取图像：
- 通过V4L2接口访问摄像头设备
- 可配置分辨率、帧率等参数
- 支持从YAML文件加载相机标定参数

## 参数对比

### 激光雷达参数
| 参数 | 集成前(模拟) | 真实X3 Pro | 当前实现 |
|------|-------------|------------|----------|
| range_min | 0.01m | 0.01m | 0.01m |
| range_max | 25.0m | 25.0m | 25.0m |
| scan_freq | 10.0Hz | 5-12Hz | 10.0Hz |
| angle_res | 0.33° | 0.33° | 0.33° |
| baudrate | N/A | 230400 | 230400 |
| isSingleChannel | N/A | false | false |

### 摄像头参数
| 参数 | 模拟值 | 真实C100 | 当前实现 |
|------|--------|----------|----------|
| width | 640 | 640 | 640 |
| height | 480 | 480/720 | 640x480 |
| fps | 30.0 | 30.0 | 30.0 |

## 验证步骤

1. 启动传感器节点
2. 检查话题输出：
   - `/scan` - 激光雷达数据
   - `/camera/image_raw` - 图像数据
3. 验证数据质量和频率
4. 连接真实硬件并验证功能

## 总结

当前实现完成了YDLidar X3 Pro的真实硬件集成，使用官方SDK替代了之前的模拟实现。摄像头驱动也使用真实的OpenCV实现，直接从USB摄像头设备读取数据。这使得整个项目仅依赖于zhongxi文件夹即可运行SLAM系统，不再需要ori_resource目录。